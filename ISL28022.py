# !/usr/bin/python3
#
# Copyright (c) 2023 Dennis Glatting
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#
# The language used in my comments are in keeping with RFC-2119.
#
#
# Class to implement the Curious Electric DC Power Monitor ISL28022 in
# Python. The ISL28022 data sheet can be found here:
#  https://www.renesas.com/us/en/document/dst/isl28022-datasheet
# As such, defaults are chosen and this code targeted to this
# board. Most of the existing code I found on the Internet, both
# Python and C++, sucked; so I'm adding this to the world's
# suck-y-ness.
#
# This class depends on busio from Adafruit blinka. To install:
#  pip3 install adafruit-blinka
# Why? Because the Curious Electric board will be used in an
# Adafruit/Seeed/Raspberry PI project.
#
# This code does not allow the configuration of the ISL28022 to be
# changed on the fly, although it could. Most IoT use cases are to set
# up a device then run with it.
#
# Many functions of the ISP28022 here are untested, such as triggered
# conversions. My use case is in a very active i2c environment where
# power is polled at regular 5sec intervals where triggering MAY make
# sense but I simply chose to read whatever is in the registers at
# that moment. Interrupts and an external oscillator IS NOT supported
# in the Curious Electric implementation of the ISL28022, which I
# wouldn't use anyway.
#
# Just a note: In my lab, my primary revision control system is RCS
# rather than git. RCS is "old school," as am I.
#
# $Log: ISL28022.py,v $
# Revision 1.16  2023/01/16 07:54:31  root
# Fixed a fundamental spec/math problem when we're dealing with mV/V
# and Ohm/mOhm. This will fix current & power calc.
#
# Revision 1.15  2023/01/16 06:48:57  root
# * Changed the debug flag to be a value than a boolean to create
#   levels of debug info.
# * Defined the shunt and bus voltages as a float to avoid later
#   casting.
# * Changed "address" to "addr" to be more consistent with other IoT
#   code.
# * Added a "pretty" print for the configuration register to made it's
#   debug more useful.
# * Commented out the main test loop.
#
# Revision 1.14  2023/01/13 22:10:49  root
# * Updates to comments.
# * Moved the delay loop in main().
#
# Revision 1.13  2023/01/13 21:26:58  root
# Yet another minor text change.
#
# Revision 1.12  2023/01/13 21:26:19  root
# Very minor text change.
#
# Revision 1.11  2023/01/13 21:14:40  root
# Current calc was off by 1,000. Resistor spec is in *milli*
# Ohms. This fixed an apparent current measurement instability. Opps.
#
# Revision 1.10  2023/01/13 08:39:29  root
# Added a "debug" parameter to several functions independent of the
# class debug flag. This was done to reduce the overall debug output
# noise so to focus a debugging effort.
#
# Revision 1.9  2023/01/13 07:47:02  root
# Documentation changes.
#
# Revision 1.8  2023/01/13 07:40:37  root
# Equation variable values I used in _CurrentLSB and _CalRegVal were
# reversed. This resulted in Current instability. There MAY still be
# a problem with the Current measurement.
#
# Revision 1.7  2023/01/13 02:27:26  root
# * Minor changes: both text and testing.
#
# Revision 1.6  2023/01/13 01:33:44  root
# * A few pretty print clean-ups.
# * Updated delay in main loop.
#
# Revision 1.5  2023/01/12 22:44:02  root
# * Added ident.
# * Decided to delete run-time reconfiguration - that's not commonly
#   how these devices are used and probably a lot of wasted and
#   complex code.
# * Bug fixes.
#
# Revision 1.4  2023/01/12 09:27:37  root
# Syntactical fix.
#
# Revision 1.3  2023/01/12 09:23:40  root
# Added more functions. Cleaned up areas. Fixed bugs.
#
# Revision 1.2  2023/01/12 07:56:59  root
# Added more functionality and debugging.
#
# Revision 1.1  2023/01/12 04:21:04  root
# Initial revision
#

ident = "$Id: ISL28022.py,v 1.16 2023/01/16 07:54:31 root Exp root $"


import board
import math
import time


class ISL28022( object ):

    def __init__( self, i2c, # i2c bus address.
                  addr=0x40,
                  # Full scale bus voltage: 16, 32, or 64v (there's a
                  # quiz later).
                  full_scale=16.0,
                  # Voltage across the shunt in mV (i.e., milli Volts).
                  shunt_voltage=320.0,
                  # Shunt resistor in mOhms (i.e., milli Ohms). 
                  shunt_resistor=5.0,
                  # Bus A/D averaging. Averaging of "0" means no
                  # averaging.
                  bavg=0,
                  # Shunt A/D averaging. Averaging of "0" means no
                  # averaging.
                  savg=0,
                  # Operational mode. Default is Shunt+Bus is
                  # continuous. Be aware of trigger mode because the
                  # data sheet doesn't clearly explain how that
                  # works. Therefore, this class doesn't(?) support
                  # trigger mode.
                  mode=0b111,
                  debug=0 ):

        self._config = ( mode & 0b111 )
        
        # Remember init values.
        self._i2c           = i2c
        self._addr          = addr
        self._calib         = 0
        self._bus_voltage   = full_scale
        self._shunt_voltage = shunt_voltage
        self._shunt_r       = shunt_resistor
        self._bavg          = bavg
        self._savg          = savg
        self._debug         = debug
        
        # The device's registers (TABLE 2).
        #  (Please give me c++ static const)
        self._register_config                  = 0x00
        self._register_shunt_voltage           = 0x01
        self._register_bus_voltage             = 0x02
        self._register_power                   = 0x03
        self._register_current                 = 0x04
        self._register_calibration             = 0x05
        self._register_shunt_voltage_threshold = 0x06
        self._register_bus_voltage_threshold   = 0x07
        self._register_dcs_interrupt_status    = 0x08
        self._register_aux_control             = 0x09

        # Configuration register bits (TABLE 3).
        #  (Please give me c++ static const)
        self._config_bits = { "RST":   0b1000000000000000,
                              "BRNG1": 0b0100000000000000,
                              "BRNG0": 0b0010000000000000,
                              "PG1":   0b0001000000000000,
                              "PG0":   0b0000100000000000,
                              "BADC3": 0b0000010000000000,
                              "BADC2": 0b0000001000000000,
                              "BADC1": 0b0000000100000000,
                              "BADC0": 0b0000000010000000,
                              "SADC3": 0b0000000001000000,
                              "SADC2": 0b0000000000100000,
                              "SADC1": 0b0000000000010000,
                              "SADC0": 0b0000000000001000,
                              "MODE2": 0b0000000000000100,
                              "MODE1": 0b0000000000000010,
                              "MODE0": 0b0000000000000001
        }

        # Save on future some math.
        #  (Please give me c++ static const)
        self._pow_table16 = [ 0b0000000000000001,
                              0b0000000000000010,
                              0b0000000000000100,
                              0b0000000000001000,
                              0b0000000000010000,
                              0b0000000000100000,
                              0b0000000001000000,
                              0b0000000010000000,
                              0b0000000100000000,
                              0b0000001000000000,
                              0b0000010000000000,
                              0b0000100000000000,
                              0b0001000000000000,
                              0b0010000000000000,
                              0b0100000000000000,
                              0b1000000000000000 ]
        
        # Checks
        assert self._bus_voltage in [ 16.0, 32.0, 60.0 ], "Full scale range set error"
        assert self._shunt_voltage in [ 40.0, 80.0, 160.0, 320.0 ], "Shunt voltage set error"
        assert self._bavg in [ 0, 1, 2, 4, 8, 16, 32, 64, 128 ], "Bus averaging set error"
        assert self._savg in [ 0, 1, 2, 4, 8, 16, 32, 64, 128 ], "Shunt averaging set error"
        assert not( mode & ~0b111 ), "Excess mode bits"
        
        # Is the device on the bus?
        assert self._i2c.try_lock(), "The i2c bus should not be locked"
        _addrs = self._i2c.scan()
        self._i2c.unlock()
        
        assert self._addr in _addrs, "Address {:} not found on bus. YMMV".format( self._addr )

        # Lets reset the device. Various driver implementations have
        # the reset bit set when they initialize the device with the
        # running configuration however the documentation says
        # "initializes the registers to their default values and
        # performs a system calibration." The documentation, I assume,
        # says the system performs a calibration to the default values
        # and not the intended values?
        _buf = bytearray([ self._register_config,
                           (( self._config_bits[ "RST" ] & 0xff00 ) >> 8 ),
                            ( self._config_bits[ "RST" ] & 0x00ff )])
        self._write( _buf )
    
        # Set up and set the configuration bit register.
        self._determine_configuration_register()

        # Write the configuration to the device.
        _buf = bytearray([    self._register_config,
                           (( self._config & 0xff00 ) >> 8 ),
                            ( self._config & 0x00ff )])
        self._write( _buf )
        if self._debug:
            print( "Configuration reg: {0} ({1})".format( _buf, self._pretty_print_config()))
	
        # Calculate the calibration values and store them away.
        self._CurrentFS  = self._mV2V( self._shunt_voltage ) / self._mOhm2Ohm( self._shunt_r )
        self._CurrentLSB = self._CurrentFS / math.pow( 2, self._resolution( "PG" ))
        self._CalRegVal  = ( math.pow( 2, 12 ) * 0.000010 ) / ( self._CurrentLSB * self._mOhm2Ohm( self._shunt_r ))

        print( "CurrentFS:  %.6f" % self._CurrentFS )
        print( "CurrentLSB: %.6f" % self._CurrentLSB )
        print( "CalRegVal:  %.6f" % self._CalRegVal )

        print( math.pow( 2, 12 ) * 0.000010, self._shunt_r )
        
        self._calib = ( int( self._CalRegVal ) & 0xfffe )

        if self._debug:
            print( " Shunt Voltage:", self._shunt_voltage, "(mV)\n",
                   "Shunt Resistence:", self._shunt_r, " (mOhm)\n",
                   "CurrentFS:", self._CurrentFS, "\n",
                   "CurrentLSB:", self._CurrentLSB, "\n",
                   "CalRegVal:", self._calib, "hex:", hex( self._calib ))

        # Electrical Spec: Vbus_step.
        self._VbusLSB = 0.004 # Volts
        
        # Write the calibration to the device.
        _buf = bytearray([ self._register_calibration,
                           (( self._calib & 0xff00 ) >> 8 ),
                            ( self._calib & 0x00ff )])
        self._write( _buf )
        if self._debug:
            print( "Calibration reg:", _buf )

    def _determine_configuration_register( self ):

        # Set up the configuration register against passed variables
        # (i.e., build the bit array suitable to be written to the
        # device but DO NOT set the device, rather merely stuff the
        # bit array away). Other pieces of information will later be
        # interpreted from this variable, such as A/D precision.
        
        _cfg = self.modes()

        if self._bus_voltage == 32.0:
            _cfg |= self._config_bits[ "BRNG0" ]
        if self._bus_voltage == 60.0:
            _cfg |= self._config_bits[ "BRNG1" ]

        if self._shunt_voltage == 80.0:
            _cfg |= self._config_bits[ "PG0" ]
        if self._shunt_voltage == 160.0:
            _cfg |= self._config_bits[ "PG1" ]
        if self._shunt_voltage == 320.0:
            _cfg |= self._config_bits[ "PG0" ]
            _cfg |= self._config_bits[ "PG1" ]

        if self._bavg == 0:
            # The resolution for the bus voltage is either 12, 13, or
            # 14 bits. A resolution of 15 bits is not supported by the
            # device. See documentation Tables 12, 13, and 14.
            if self._bus_voltage == 60.0: # 14 bits
                _cfg |= self._config_bits[ "BADC1"]
            if self._bus_voltage == 32.0: # 13 bits
                _cfg |= self._config_bits[ "BADC0"]
            if self._bus_voltage == 16.0: # 12 bits
                pass
        else:
            _cfg |= self._config_bits[ "BADC3" ]
            if self._bavg == 1:
                pass
            if self._bavg == 2:
                _cfg |= self._config_bits[ "BADC0" ]
            if self._bavg == 4:
                _cfg |= self._config_bits[ "BADC1" ]
            if self._bavg == 8:
                _cfg |= self._config_bits[ "BADC0" ]
                _cfg |= self._config_bits[ "BADC1" ]
            if self._bavg == 16:
                _cfg |= self._config_bits[ "BADC2" ]
            if self._bavg == 32:
                _cfg |= self._config_bits[ "BADC2" ]
                _cfg |= self._config_bits[ "BADC0" ]
            if self._bavg == 64:
                _cfg |= self._config_bits[ "BADC2" ]
                _cfg |= self._config_bits[ "BADC1" ]
            if self._bavg == 128:
                _cfg |= self._config_bits[ "BADC2" ]
                _cfg |= self._config_bits[ "BADC1" ]
                _cfg |= self._config_bits[ "BADC0" ]

        if self._savg == 0:
            # The resolution for the shunt voltage is either 13, 14,
            # or 15 bits. See documentation Tables 8, 9, 10, and 11.
            if self._shunt_voltage == 320.0: # 15 bits
                _cfg |= self._config_bits[ "SADC0"]
                _cfg |= self._config_bits[ "SADC1"]
            if self._shunt_voltage == 160.0: # 14 bits
                _cfg |= self._config_bits[ "SADC1"]
            if self._shunt_voltage == 80.0: # 13 bits
                _cfg |= self._config_bits[ "SADC0"]
        else:
            _cfg |= self._config_bits[ "SADC3" ]
            if self._bavg == 1:
                pass
            if self._savg == 2:
                _cfg |= self._config_bits[ "SADC0" ]
            if self._savg == 4:
                _cfg |= self._config_bits[ "SADC1" ]
            if self._savg == 8:
                _cfg |= self._config_bits[ "SADC0" ]
                _cfg |= self._config_bits[ "SADC1" ]
            if self._savg == 16:
                _cfg |= self._config_bits[ "SADC2" ]
            if self._savg == 32:
                _cfg |= self._config_bits[ "SADC2" ]
                _cfg |= self._config_bits[ "SADC0" ]
            if self._savg == 64:
                _cfg |= self._config_bits[ "SADC2" ]
                _cfg |= self._config_bits[ "SADC1" ]
            if self._savg == 128:
                _cfg |= self._config_bits[ "SADC2" ]
                _cfg |= self._config_bits[ "SADC1" ]
                _cfg |= self._config_bits[ "SADC0" ]

        # Lets squirrel that away.
        self._config = _cfg

        
    def _resolution( self, w ):

        # Return the current bit resolution of the bus voltage or
        # shunt voltage against the configuration register.
        
        assert w in [ "BRNG", "PG" ]
        
        _res = 0
        _bits = self._config & self._mask([  w + "1", w + "0" ])

        if w == "PG":

            if _bits == 0:
                _res = 12
            if _bits == self._mask([ w + "0" ]):
                _res = 13
            if _bits == self._mask([ w + "1" ]):
                _res = 14
            if _bits == self._mask([ w + "1", w + "0" ]):
                _res = 15

        if w == "BRNG":

            if _bits == 0:
                _res = 12
            if _bits == self._mask([ w + "0" ]):
                _res = 13
            if _bits == self._mask([ w + "1" ]):
                _res = 14
            if _bits == self._mask([ w + "1", w + "0" ]):
                _res = 14
                
        return _res
   

    def _converson_delay( self, reg ):

        # When a conversion is requested, you have to WAIT.

        #  (Please give me c++ static const)
        _conversion_times = [ 0.000080, 0.000146, 0.000284, 0.000559,
                              0.001110, 0.002210, 0.004410, 0.008810,
                              0.017610, 0.035210, 0.070410 ]
        
        _delay = 0.0
        _bits = 0

        # If you have kids, now is the time to hide their eyes.
        if reg == self._register_shunt_voltage:

            _bits = self._register_config & self._mask([ "SADC3", "SADC2", "SADC1", "SADC0" ])

            if _bits == 0:
                _delay = _conversion_times[ 0 ]
            if _bits == self._mask([ "SADC0" ]):
                _delay = _conversion_times[ 1 ]
            if _bits == self._mask([ "SADC1" ]):
                _delay = _conversion_times[ 2 ]
            if _bits == self._mask([ "SADC1", "SADC0" ]):
                _delay = _conversion_times[ 3 ]
            if _bits == self._mask([ "SADC3", "SADC0" ]):
                _delay = _conversion_times[ 4 ]
            if _bits == self._mask([ "SADC3", "SADC1" ]):
                _delay =	_conversion_times[ 5 ]
            if _bits == self._mask([ "SADC3", "SADC1", "SADC0" ]):
                _delay = _conversion_times[ 6 ]
            if _bits == self._mask([ "SADC3", "SADC2" ]):
                _delay = _conversion_times[ 7 ]
            if _bits ==	self._mask([ "SADC3", "SADC2", "SADC0" ]):
                _delay = _conversion_times[ 8 ]
            if _bits == self._mask([ "SADC3", "SADC2", "SADC1" ]):
                _delay = _conversion_times[ 9 ]
            if _bits == self._mask([ "SADC3", "SADC2", "SADC1", "SADC0" ]):
                _delay = _conversion_times[ 10 ]
                
        if reg == self._register_bus_voltage:

            _bits = self._config & self._mask([ "BADC3", "BADC2", "BADC1", "BADC0" ])

            if _bits == 0:
                _delay = _conversion_times[ 0 ]
            if _bits == self._mask([ "BADC0" ]):
                _delay = _conversion_times[ 1 ]
            if _bits == self._mask([ "BADC1" ]):
                _delay = _conversion_times[ 2 ]
            if _bits == self._mask([ "BADC1", "BADC0" ]):
                _delay = _conversion_times[ 3 ]
            if _bits == self._mask([ "BADC3", "BADC0" ]):
                _delay = _conversion_times[ 4 ]
            if _bits == self._mask([ "BADC3", "BADC1" ]):
                _delay = _conversion_times[ 5 ]
            if _bits == self._mask([ "BADC3", "BADC1", "BADC0" ]):
                _delay = _conversion_times[ 6 ]
            if _bits == self._mask([ "BADC3", "BADC2" ]):
                print( "foo" )
                _delay = _conversion_times[ 7 ]
            if _bits == self._mask([ "BADC3", "BADC2", "BADC0" ]):
                _delay = _conversion_times[ 8 ]
            if _bits == self._mask([ "BADC3", "BADC2", "BADC1" ]):
                _delay = _conversion_times[ 9 ]
            if _bits == self._mask([ "BADC3", "BADC2", "BADC1", "BADC0" ]):
                _delay = _conversion_times[ 10 ]

        return _delay

    def _mask( self, flags ):

        # Weird wacky masking of the configuration register
        # bits. Don't ask questions: you don't want to know.
        #
        # What is intended here is a list of mask bits are passed (see
        # _config_bits) and a bit array is formed against the
        # configuration register. Unrequested bits are 0.
        
        _bits = 0
        for _i in flags:
            _bits |= self._config_bits[ _i ]

        return _bits

    def _buf_to_int( self, buf ):

        # Convert the two byte value in the buffer to a 16 bit
        # integer. The assumption is buf[0] is the MSB and buf[1] is
        # the LSB.

        assert len( buf ) >= 2, "Not enough data in buffer"

        return int(( buf[0] << 8 ) | buf[1])
    
    def _write( self, buf ):

        if self._debug > 1:
            print( "write:", buf, ' ', end='' )

        assert self._i2c.try_lock(), "The i2c bus should not be locked"

        _nacks = self._i2c.writeto( self._addr, buf )

        self._i2c.unlock()
        
        if self._debug > 1:
            print( "nacks:", _nacks )

            
    def _readreg16( self, reg ):

        # Read a 16 bit value from the device's register.
        
        _wbuf = bytearray([ reg ])
        _rbuf = bytearray( 2 )

        if self._debug > 1:
            print( "readreg16 w:", _wbuf, ' ', end='' )

        assert self._i2c.try_lock(), "The i2c bus should not be locked"

        _ret = self._i2c.writeto_then_readfrom( self._addr, _wbuf, _rbuf )
        
        self._i2c.unlock()

        assert len( _rbuf ) == 2, "16 bits not returned from reg: {:}".format( reg )

        if self._debug > 1:
            print( "r:", _rbuf, "ret:", _ret )
            
        return _rbuf

    def _twos_complement16( self, val16, bits ):

        # Return a two's complement of a value whose *total* bit
        # length is "bits" with the MSB the sign bit. There are a lot
        # of online "tricks" but I chose to write my own (i.e., I'm
        # not a fan of tricks).
        
        _neg  =	val16 & self._pow_table16[ bits ]

        _v = 0.0
        if _neg:
            _v = -1

        for n in range( 0, bits - 1 ):
            if _neg:
                if not( val16 & self._pow_table16[ n ]):
                    _v += -self._pow_table16[ n ]
            else:
                if val16 & self._pow_table16[ n ]:
                    _v += self._pow_table16[ n ]

        return _v

    def _pretty_print_config( self ):

        # It is a pain to look at the 16bit configuration register in
        # hex when it's meaningful in another format.
        
        _ret = ""

        _ret += self._zero_or_one( self._config & self._config_bits[ "RST" ]) 
        _ret += " "

        _ret += self._zero_or_one( self._config & self._config_bits[ "BRNG1" ])
        _ret +=	self._zero_or_one( self._config & self._config_bits[ "BRNG0" ])
        _ret +=	" "

        _ret +=	self._zero_or_one( self._config & self._config_bits[ "PG1" ])
        _ret += self._zero_or_one( self._config & self._config_bits[ "PG0" ])
        _ret += " "

        _ret += self._zero_or_one( self._config & self._config_bits[ "BADC3" ])
        _ret +=	self._zero_or_one( self._config & self._config_bits[ "BADC2" ])
        _ret +=	self._zero_or_one( self._config & self._config_bits[ "BADC1" ])
        _ret +=	self._zero_or_one( self._config & self._config_bits[ "BADC0" ])
        _ret += " "

        _ret +=	self._zero_or_one( self._config & self._config_bits[ "SADC3" ])
        _ret += self._zero_or_one( self._config & self._config_bits[ "SADC2" ])
        _ret += self._zero_or_one( self._config & self._config_bits[ "SADC1" ])
        _ret += self._zero_or_one( self._config & self._config_bits[ "SADC0" ])
        _ret += " "

        _ret += self._zero_or_one( self._config & self._config_bits[ "MODE2" ])
        _ret += self._zero_or_one( self._config & self._config_bits[ "MODE1" ])
        _ret += self._zero_or_one( self._config & self._config_bits[ "MODE0" ])

        return _ret

    def _mOhm2Ohm( self, mOhm ):

        return float( mOhm ) / 1000.0

    def _mV2V( self, mV ):

        return float( mV ) / 1000.0

    def _zero_or_one( self, bit ):

        if bit:
            return "1"
        else:
            return "0"
        
    def initialization_delay( self ):

        # Power on delay before the first read is useful based against
        # the internal oscillator and the bus voltage conversion bits.
        
        return ( 1.0 / 500000.0 ) * self._pow_table16[ self._resolution( "PG" )]

    
    def shunt_conversion_delay( self ):

        # Delay between shunt voltage conversions in seconds.
        
        return self._converson_delay( self._register_shunt_voltage )


    def bus_conversion_delay( self ):

        # Delay between bus voltage conversions in seconds.
        
        return self._converson_delay( self._register_bus_voltage )

        
    def power_down( self ):

        self._config &= 0b1111111111111000

        _buf = bytearray([ self._register_config,
                           (( self._config & 0xff00 ) >> 8 ),
                            ( self._config & 0x00ff )])
        self._write( _buf )

        
    def adc_off( self ):

        self._config &= 0b1111111111111100
        self._config |= 0b0000000000000100

        _buf = bytearray([ self._register_config,
                           (( self._config & 0xff00 ) >> 8 ),
                            ( self._config & 0x00ff )])
        self._write( _buf )

        
    def modes( self ):

        # Return the mode bit flag.
        
        return ( self._config & 0b0000000000000111 )

    
    def shunt_voltage( self, debug=False ):

        _rbuf = self._readreg16( self._register_shunt_voltage )
        _int  = self._buf_to_int( _rbuf )
        _bits = self._resolution( "PG" )
        
        assert _bits in [ 12, 13, 14, 15 ], "Bus voltage bits: {:}".format ( _bits )

        # Electrical spec: Vshunt_step
        _vShunt = self._twos_complement16( _int, _bits )
        _ret = float( _vShunt ) * 0.000010
        
        if self._debug or debug:
            print( "Shunt voltage: _rbuf=", _rbuf, "_int=", hex( _int ), "_bits=", _bits, "_ret=", _ret )

        return _ret


    def bus_voltage( self, debug=False ):

        _rbuf = self._readreg16( self._register_bus_voltage )
        _int  = self._buf_to_int( _rbuf )
        _bits = self._resolution( "BRNG" )

        if _int & 0b0000000000000001:
            print( "Bus voltage overflow" )
    
        # There is no sign bit for the bus voltage.
        _ret = 0
    
        assert _bits in [ 12, 13, 14 ], "Bus voltage bits: {:}".format ( _bits )
        
        if _bits == 12:
            _ret = ( _int & 0b0111111111111000 ) >> 3
        if _bits == 13:
            _ret = ( _int & 0b1111111111111000 ) >> 3
        if _bits == 14:
            _ret = ( _int & 0b1111111111111100 ) >> 2
        _ret *= self._VbusLSB

        if self._debug or debug:
            print( "Bus voltage: _rbuf=", _rbuf, "_int=", hex( _int ), "_bits=", _bits, "_ret=", _ret )

        return _ret

    
    def power( self, debug=False ):

        _rbuf = self._readreg16( self._register_power )
        _int  = self._buf_to_int( _rbuf )

        _p = 0

        for n in range( 0, 15 ):
            if _int & self._pow_table16[ n ]:
                _p += self._pow_table16[ n ]

        _ret = float( _p ) * self._CurrentLSB * self._VbusLSB * 5000.0

        if self._debug or debug:
            print( "Power: _rbuf=", _rbuf, "_int=", hex( _int ), "_p=", _p, "_ret=", _ret )
            
        return _ret
    

    def current( self, debug=False ):

        _rbuf = self._readreg16( self._register_current )
        _int  = self._buf_to_int( _rbuf )

        _c = self._twos_complement16( _int, 15 )
        _ret = float( _c ) * self._CurrentLSB
        
        if self._debug or debug:
            print( "Current: _ruf=", _rbuf, "_int=", hex( _int ), "_c=", _c, "_ret=", _ret, "fake:", ((( _int / 32768.0) * 320.0 ) * 5.0 ))

        return _ret

    
    # Diagnostic, mostly.
    def other_regs( self ):

        _rbuf = self._readreg16( self._register_calibration )
        print( "Calibration:         ", hex( self._buf_to_int( _rbuf )))

        _rbuf = self._readreg16( self._register_shunt_voltage_threshold )
        print( "Shunt threshold:     ", hex( self._buf_to_int( _rbuf )))
               
        _rbuf = self._readreg16( self._register_bus_voltage_threshold  )
        print( "Bus threshold:       ", hex( self._buf_to_int( _rbuf )))

        _rbuf = self._readreg16( self._register_dcs_interrupt_status )
        print( "DCS interrupt status:", hex( self._buf_to_int( _rbuf )))
        
        _rbuf = self._readreg16( self._register_aux_control )
        print( "AUX control:         ", hex( self._buf_to_int( _rbuf )))
    

i2c = board.I2C()

isl = ISL28022( i2c, bavg=8 )



#print( ident )
#print()
#
## A pause for the cause.
#print( "Initialization delay: %.6f us" % isl.initialization_delay())
#print( "Bus conversion delay: %.6f us" % isl.bus_conversion_delay())
#print()
#
#time.sleep( isl.initialization_delay())
#
#
#while True:
#
#    print( "Bus Voltage:   %+2.4f" % isl.bus_voltage())
#    print( "Shunt Voltage: %+2.4f" % isl.shunt_voltage())
#    print( "Current:       %+2.4f" % isl.current())
#    print( "Power:         %+2.4f" % isl.power())
#    print()
#    
#    isl.other_regs()
#    print()
#
#    time.sleep( max( isl.bus_conversion_delay(),
#                     isl.shunt_conversion_delay(), 0.100 ))
    


#  LocalWords:  busio Adafruit blinka MSB LSB IoT mOhm
