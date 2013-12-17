# python setupFG.py frequency amplitude offset waveform output
# frequency in Hz
# amplitude Vpp
# offset V
# waveform must be one of [sine,square,ramp,noise,arbitrary,dc]
# output must be one of [on,off]

# Choose Hardware
# 0 = DS345, 1 = Tektronix 3021B , 2 = Agilent (MODEL?)
fgtype=0

# Choose Interface (i.e., 'serial', 'gpib')
fgint='serial'
#fgint='gpib'

# Specify path of interace (i.e., '/dev/ttyUSB0')
fgloc='/dev/ttyUSB0'
#fgloc=<pad> # GPIB, i.e., fgloc=9 

############ DO NOT EDIT BELOW THIS LINE ###################
from sys import argv
if fgint =='serial':
    import serial
elif fgint =='gpib':
    from gpibinter import ibdev,ibwrt
else:
    print 'Specified interface not supported!'

if len(argv) <= 1:
    print "setupFG.py frequency amplitude offset waveform output"
    print "waveform must be one of [sine,square,ramp,noise,arbitrary,dc]"
    print "output must be one of [on,off]"
    exit(1)

freq=argv[1]
amp=argv[2]
off=argv[3]
wave=argv[4]
out=argv[5]

class ctrl:
    #def __init__(self,parent=None,type=fgint,loc=fgloc):
    def write(self,cmd,type=fgint,loc=fgloc):
        if type=='gpib':
            self.id=ibdev(pad=loc)
            ibwrt(self.id, cmd)
        if type=='serial':
            self=serial.Serial(loc,timeout=1,baudrate=9600)
            self.write(cmd)            
fg=ctrl()

######### DS345 STUFF (fgtype=0) ########
if fgtype == 0: # ds345

    # Define values #
    freqstr = str(freq)

    if wave == 'sine' or wave == 'SINE':
        wavestr = '0'
    elif wave == 'square' or wave == 'SQUARE':
        wavestr = '1'
    elif wave == 'triangle' or wave == 'TRIANGLE':
        wavestr = '2'
    elif wave == 'ramp' or wave == 'RAMP':
        wavestr = '3'
    elif wave == 'noise' or wave == 'NOISE':
        wavestr = '4'
    elif wave == 'arbitrary' or wave == 'ARBITRARY':
        wavestr = '5'
    elif wave == 'dc' or wave == 'DC':
        # doesn't exist for ds345
        # command DC level through offset only
        wavestr = '0'
        freqstr = '0'
        ampstr = '0'
    else:
        print 'invalid wave type'
        exit(1)

    # output doesn't exist for ds345
    # for off set amp, freq and off = 0
    if out == 'ON' or out == 'on':
        ampstr = str(amp)
    elif out == 'OFF' or out == 'off':
        ampstr = '0'
        freqstr = '0'
        offstr = '0'
    else:
        print 'invalid output type'
        exit(1)

    offstr = str(off)

    # Set Waveform #
    cmdstr = 'FUNC ' + wavestr
    fg.write('%s\n'%cmdstr)

    # Set Frequency #
    cmdstr = 'FREQ ' + freqstr
    fg.write('%s\n'%cmdstr)

    # Set Amp #
    cmdstr = 'AMPL ' + ampstr + 'VP'
    fg.write('%s\n'%cmdstr)

    # Set Off #
    cmdstr = 'OFFS ' + offstr
    fg.write('%s\n'%cmdstr)
