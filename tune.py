#/usr/bin/env python
import umuxlib, os, time
from sys import argv
from numpy import *
from scipy import optimize
from datetime import date
from scipy.io import *
from pylab import *

#################################################
# USER DEFS:
#################################################
freqFile='/data/raw/default_freqs.txt'
loSpanShort=1e6 #1
loSpanLong=2e6  #2
voff=[i*0.1 for i in range(10)]
extraPlots=0
freqoff=-0.001e9  #offset to add to default_freqs.txt
#################################################

if(len(argv)<2):
    t=date.today()
    saveDir="/data/raw/" + t.strftime("%Y%m%d") + "/"
else:
    saveDir=argv[1]+"/"
# if it doesn't exist create it
try:
    os.stat(saveDir)
except:
    os.mkdir(saveDir)

if(len(argv)<3):
    plot2screen=0
else:
    plot2screen=argv[2]

startTime=time.time()

um=umuxlib.util()
um.loadFreqsAttens(freqFile)
um.freqs=um.freqs.copy()+freqoff
# for 35 resonator tones the power is low enough to not saturate the
# josephson junction, and atten_out = 0 is fine.  For fewer channels
# the power per channel is higher and it may be necessary to optimize
if len(um.freqs)>5:
    atten_out=array([0])
else:
    atten_out=array([0,2,4,6,8,10,12,14,16,18,20])
#override for now
atten_in=0
atten_out=array([0])
print "using atten_out = %d, atten_in = %d" %(atten_out[0],atten_in)

um.saveDir=saveDir
um.loSpan=loSpanLong
um.atten_in=atten_in
um.atten_out=array([0])  # don't do an atten sweep for first run through

# program the output waveform LUT with the default freqs
um.defineLUTs()

# before starting make sure the flux ramp is off
um.setupFG(0,0,0,'ramp','off')

# make sure the centers are cleared out
print "clearing centers..."
um.iq_centers=[0.+0.j]*256
um.translateLoops()

# do LO sweep at atten_out_start
print "LO sweep for s21 vs freq for rough resonance detection..."
um.sweepLO(um.freqs,um.loFreq,loSpanShort,atten_in,atten_out)

figure(1); 
# make a mag(s21) vs freq plot:
print "plotting s21 vs freq..."
for ii in range(um.Nfreqs):
    semilogy(array(um.f_span[ii])/1.e+9, (um.I[ii,:]**2 + um.Q[ii,:]**2)**.5,'k.-');
axis('tight');
ax=gca(); xmin,xmax=ax.get_ylim();
semilogy([um.loFreq/1.e+9,um.loFreq/1.e+9],[xmin,xmax],'r--');
xlabel('freq (GHz)'); ylabel('mag(s21)'); draw(); draw();
title('mag(S21) vs frequency');
if plot2screen: 
    show(); 
else:
    fn='%s/S21vFreq.png' %saveDir
    savefig(fn)
    os.chmod(fn,0777)

# plot the raw IQ loops:
print "plotting raw IQ loops..."
clf()
for ii in range(um.Nfreqs):
    plot(um.I[ii,:],um.Q[ii,:],'k.-');
    plot(um.iq_centers[ii].real,um.iq_centers[ii].imag,'ro');
xlabel('I (adu)'); ylabel('Q (adu)'); title('raw IQ loops'); axis('equal'); draw(); draw();
if plot2screen: 
    show(); 
else: 
    fn='%s/IQloops_raw.png' %saveDir
    savefig(fn)
    os.chmod(fn,0777)

# center the IQ loops on the origin
um.translateLoops()
print "sweeping to verify roughly centered IQ loops..."
um.sweepLO(um.freqs,um.loFreq,loSpanShort,atten_in,atten_out)
clf()
for ii in range(um.Nfreqs):
    plot(um.I[ii,:],um.Q[ii,:],'k.-');
    #plot(um.iq_centers[ii].real,um.iq_centers[ii].imag,'ro');
    xlabel('I (adu)'); ylabel('Q (adu)'); title('centered IQ loops'); axis('equal'); draw(); draw()
    if plot2screen:
        show(); 
    else: 
        fn='%s/IQloops_centered.png' %saveDir
        savefig(fn)
        os.chmod(fn,0777)

# now set up atten settings with desired values for power sweeps
um.atten=atten_out
Noff=len(voff)
Natt=len(atten_out)
Nch=um.Nfreqs
Nfr=int(loSpanShort/um.df)

Iall=zeros([Nch,Nfr,Natt,Noff])
Qall=zeros([Nch,Nfr,Natt,Noff])
print "doing power sweeps..."
for ii in range(Noff):
    um.setupFG(0,0,voff[ii],'dc','on')
    time.sleep(0.1)
    um.sweepLO(um.freqs,um.loFreq,loSpanShort,atten_in,atten_out)
    Iall[:,:,:,ii]=um.I
    Qall[:,:,:,ii]=um.Q
    
# tuning algorithm to select probe freq and power/atten
# indices of Iall/Qall:  (ch,freq,atten,voff)

# now there should only be signal in the ph direction, none in mag direction:
ph=arctan2(Qall,Iall)   # in radians
ph=unwrap(ph)
mag=(Iall**2+Qall**2)**0.5

# for each ch find the frequency cut that maximizes modulation depth
# actually we want to maximize signal to noise, signal=mod depth, noise=std(phase_timestream)
print "finding optimal readout freq for each channel..."
mod=zeros([Nch,Natt,Nfr])
for ii in range(Nch):
    for jj in range(Natt):
        for kk in range(Nfr):
            # may want to fit instead if really noisy
            mod[ii,jj,kk]=ph[ii,kk,jj,:].max()-ph[ii,kk,jj,:].min()

# fit response humps and choose best probe freq and atten
# for now just use atten out = 0.  doesn''t seem to matter
# use a parabolic fit for now.  doesn't really fit the model well
# come up with another expression / model in the future
print "determining resonant frequency and attenuation..."
freqFits=zeros([Nch,Natt])
modDepth=zeros([Nch,Natt])
badRes=zeros([Nch])
modThresh=0.5
# the best method is probably fit a parabola over a small span near the peak
fitfunc = lambda pp,xx:  pp[0]*(xx[0]-pp[1])**2 + pp[2]
errfunc = lambda pp,xx:  fitfunc(pp,xx)-xx[1]
ptrng=10   # +/- number of points around the max to use for p2 fit
# set up plot stuf
nrow=9
ncol=4
fs=8
xlims=[-0.5,0.5]
ylims=[0,3]
clf();
for ii in range(Nch):
    for jj in range(Natt):
        mxind=argmax(mod[ii,jj,:])
        if mxind<ptrng:
            print "warning! max response near boundary of sweep!"
            mxind=ptrng
        xx=[array(um.f_span[ii]), transpose(mod[ii,jj,:])]  
        xxx=[xx[0][(mxind-ptrng):(mxind+ptrng)],xx[1][(mxind-ptrng):(mxind+ptrng)]]
        # determine if the resonator doesn't respond to the flux ramp
        if max(xxx[1]) < modThresh:
            badRes[ii]=1
        # initial guess
        pp0=[-1.e-12,mean(xxx[0]),max(xxx[1])] #p2 fit around range of res freq
        ppOut,success=optimize.leastsq(errfunc, pp0[:], args=(xxx),maxfev=2000)
        freqFits[ii,jj]=ppOut[1]
        modDepth[ii,jj]=max(fitfunc(ppOut,xxx))
        # show response
        subplot(nrow,ncol,ii+1).plot((xx[0]-mean(xx[0]))/1.e+6,xx[1],'-'); 
        # show fit
        subplot(nrow,ncol,ii+1).plot((xxx[0]-mean(xx[0]))/1.e+6,fitfunc(ppOut,xxx),'r-');
        # show selected tone
        subplot(nrow,ncol,ii+1).plot((freqFits[ii,jj]-mean(xx[0]))/1.e+6,xx[1][argmax(xx[1])],'gx');
        ax=gca();
        #text(0.95,0.95,str(ii),'matplotlib',horizontalalignment='center',verticalalignment='center',transform = ax.transAxes)
        # indicate bad resonators
        if badRes[ii]:
            subplot(nrow,ncol,ii+1).plot([xlims[0],xlims[1]],[ylims[0],ylims[1]],'r-');
            subplot(nrow,ncol,ii+1).plot([xlims[0],xlims[1]],[ylims[1],ylims[0]],'r-');
        if (ii != 32):
            ax.yaxis.set_ticklabels(''); ax.xaxis.set_ticklabels('');
        else:
            ax.xaxis.label.set_fontsize(fs); ax.yaxis.label.set_fontsize(fs)
            for tick in ax.xaxis.get_major_ticks():
                tick.label1.set_fontsize(fs)
            for tick in ax.yaxis.get_major_ticks():
                tick.label1.set_fontsize(fs)
            xlabel('delta freq (MHz)',fontsize=fs); ylabel('mod depth (rad)',fontsize=fs); 
        ylim((ylims[0],ylims[1]));
        xlim((xlims[0],xlims[1]));

suptitle('mod depth vs probe frequency');
draw(); draw();
if plot2screen: 
    show(); 
else: 
    fn='%s/frmMagfit.png' %saveDir
    savefig(fn)
    os.chmod(fn,0777)

# find the attenuation setting that maximizes the modulation depth
freqUse=zeros([Nch])
attenUse=zeros([Nch])
for ii in range(Nch):
    induse=argmax(modDepth[ii,:])
    freqUse[ii]=freqFits[ii,induse]
    # if the resonator doesn't respond to flux ramp
    # set attenuation really high unless it's a dark SQUID or res
    # do this to preserve the output power of DAC.  DO THIS LATER!!!!
    attenUse[ii]=atten_out[induse]

# now write these to a new freqs file
freqFile=saveDir+'/tuned_freqs.txt'
um.saveFreqsAttens(um.loFreq,freqUse,attenUse,freqFile)

# program these probe frequencies
# will need to modify this if we use individ atten
um.loadFreqsAttens(freqFile)
# setting atten_out with minimum attenuation of group
# do this so that the phase centers get properly corrected
um.atten_out=array([um.attens.min()])
um.progAttens(um.atten_in,um.atten_out)
um.defineLUTs()

# center each channel's flux ramp response IQ loops
um.rotTransFRM() # rotate and translate loops

# confirm the tuning with flux ramp modulation
# don't ramp flux ramp, step and sample to get periodicity
vfrm=[i*0.01 for i in range(100)]
Ifrm=zeros([Nch,len(vfrm)])
Qfrm=zeros([Nch,len(vfrm)])
for ii in range(len(vfrm)):
    um.setupFG(0,0,vfrm[ii],'ramp','on')
    # don't go crazy, give the fg a minute to breath
    time.sleep(0.1)
    Ifrm[:,ii],Qfrm[:,ii]=um.IQread()
phfrm=unwrap(arctan2(Qfrm,Ifrm))
um.setupFG(0,0,0,'ramp','off')

# plot the flux ramp response in a subplot panel otherwise it's a mess
nrow=9
ncol=4;
fs=8
ylims=[-1.5,1.5]
xlims=[vfrm[0],vfrm[len(vfrm)-1]]
clf();
for ii in range(um.Nfreqs):
    #subplot(nrow,ncol,ii+1).plot(vfrm,phfrm[ii,:]-mean(phfrm[ii,:]));
    subplot(nrow,ncol,ii+1).plot(vfrm,phfrm[ii,:])
    #xlim((xlims[0],xlims[1]));
    ylim((ylims[0],ylims[1]));
    ax=gca();
    #title(str(ii));
    if (ii != 32):
        ax.yaxis.set_ticklabels(''); ax.xaxis.set_ticklabels('');
    else:
        ax.xaxis.label.set_fontsize(fs); ax.yaxis.label.set_fontsize(fs)
        for tick in ax.xaxis.get_major_ticks():
            tick.label1.set_fontsize(fs)
        for tick in ax.yaxis.get_major_ticks():
            tick.label1.set_fontsize(fs)
        xlabel('flux (V)',fontsize=fs); ylabel('phase (rad)',fontsize=fs); 
    if badRes[ii]:
        subplot(nrow,ncol,ii+1).plot([xlims[0],xlims[1]],[ylims[0],ylims[1]],'r-');
        subplot(nrow,ncol,ii+1).plot([xlims[0],xlims[1]],[ylims[1],ylims[0]],'r-');
suptitle('flux ramp response at probe tone');
draw(); draw();
if plot2screen: 
    show(); 
else: 
    fn='%s/frmVphi.png' %saveDir
    savefig(fn)
    os.chmod(fn,0777)

# plot the flux ramp IQ loops and verify they are centered:
clf();
for ii in range(Nch):
    plot(Ifrm[ii,:],Qfrm[ii,:]);
xlabel('I (adu)'); ylabel('Q (adu)'); title('IQ flux ramp response');
axis('equal'); 
#xlim([-150,150]); ylim([-150,150]); 
grid('on'); draw(); draw(); 
if plot2screen: 
    show(); 
else: 
    fn='%s/frmIQ.png' %saveDir
    savefig(fn)
    os.chmod(fn,0777)

# need to at least load a flat filter
if(0):
    um.loadFIRcoeffs("none")

# save data
savemat(saveDir+'tunedat.mat',{'voff':voff,'vfrm':vfrm,'Ifrm':Ifrm,'Qfrm':Qfrm,'mod':mod, \
                               'Iall':Iall,'Qall':Qall,'ph':ph,'mag':mag,'modDepth':modDepth, \
                               'badRes':badRes,'freqUse':freqUse,'attenUse':attenUse});

# print time to tune:
print "system tuned in %i seconds" %int(time.time() - startTime)

##############################################################################
##############################################################################

# some plots below to
# visualize the best readout frequency:

if extraPlots:
    # show the modulation by probe freq
    clf()
    ch=1;
    at=0;
    plot(array(um.f_span[ch])/1.e+9,ph[ch,:,at,:],'.-');
    xlabel('freq (GHz)'); ylabel('phase (rad)');
    draw(); draw();

if extraPlots:
    # show the Vphi response, by choice of freq
    # get a family of Vphi's, one for each probe freq
    clf()
    ch=1;
    at=0;
    pht=transpose(ph[ch,:,ch,:])
    for ii in range(Nfr):
        plot(voff,pht[:,ii]-pht[:,ii].mean(),'.-');
    xlabel('flux (V)'); ylabel('phase (rad)');
    draw(); draw();

# pretty obvious that the atten_out = 0 is fine
# perhaps average and fit the parabola or just fit each one
if extraPlots:
    clf();
    for ii in range(Nch):
        subplot(Nch,1,ii).plot(array(um.f_span[ii])/1.e+9,transpose(mod[ii,:,:])); 
        draw(); draw();

# plot showing best atten (row) and freq (col):
# 9 rows, 5 cols looks ok
if extraPlots:
    nrows=9; ncols=5
    clf(); draw();
    for ii in range(Nch): 
        subplot(nrows,ncols,ii+1).imshow(mod[ii,:,:])
        draw();draw();
