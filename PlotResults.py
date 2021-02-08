import numpy as np
import matplotlib.pyplot as plt

def load(arg,_file):
	if arg   == 'x':     k=0
	elif arg == 'y':     k=1
	elif arg == 'z':     k=2
	elif arg == 'phi':   k=3
	elif arg == 'theta': k=4
	elif arg == 'psi':   k=5
	elif arg == 'u1':    k=6
	elif arg == 'u2':    k=7
	elif arg == 'u3':    k=8
	elif arg == 'u4':    k=9
	elif arg == 'u5':    k=10
	elif arg == 'u6':    k=11
	elif arg == 'f1':    k=12
	elif arg == 'f2':    k=13
	elif arg == 'f3':    k=14
	elif arg == 'f4':    k=15
	elif arg == 'f5':    k=16
	elif arg == 'f6':    k=17
	elif arg == 'xg':    k=18
	elif arg == 'yg':     k=19
	elif arg == 'zg':     k=20
	elif arg == 'phig':   k=21
	elif arg == 'thetag': k=22
	elif arg == 'psig':   k=23
	else:
		print ('invalid argument')
		return 0
	col = []
	with open(_file+'.txt','r') as files:
		data = files.readlines()
		for line in data:
			col.append(float(line.rsplit(',')[k]))
	return col

def load2(arg,_file):
	if arg   == 'x':	k=0
	elif arg == 'xg':   k=1
	elif arg == 'dxg':  k=2
	elif arg == 'ddxg': k=3
	elif arg == 'y':	k=4
	elif arg == 'yg':   k=5
	elif arg == 'dyg':  k=6
	elif arg == 'ddyg': k=7
	elif arg == 'z':	k=8
	elif arg == 'zg':   k=9
	elif arg == 'dzg':  k=10
	elif arg == 'ddzg': k=11
	elif arg == 'phi':   	k=12
	elif arg == 'phig':   	k=13
	elif arg == 'dphig':  	k=14
	elif arg == 'ddphig': 	k=15
	elif arg == 'theta':   	k=16
	elif arg == 'thetag':   k=17
	elif arg == 'dthetag':  k=18
	elif arg == 'ddthetag': k=19
	elif arg == 'psi':   	k=20
	elif arg == 'psig':  	k=21
	elif arg == 'dpsig':	k=22
	elif arg == 'ddpsig':	k=23
	else:
		print ('invalid argument')
		return 0
	col = []
	with open(_file+'.txt','r') as files:
		data = files.readlines()
		for line in data:
			col.append(float(line.rsplit(',')[k]))
	return col

sw = input('Control: 1\nObserver: 2\n')

if sw=='1':
	x = load('x','resultsADRC')
	y = load('y','resultsADRC')
	z = load('z','resultsADRC')
	phi = load('phi','resultsADRC')
	theta = load('theta','resultsADRC')
	psi = load('psi','resultsADRC')

	xd =  1.*np.ones(len(x))
	yd = -2.*np.ones(len(x))
	zd =  2.*np.ones(len(x))
	phid = 	 0.*np.ones(len(x))
	thetad = 0.*np.ones(len(x))
	psid = 	-20.*np.ones(len(x))

	for j in range(len(phi)):
		phi[j] = (180./np.pi)*phi[j]
	for j in range(len(theta)):
		theta[j] = (180./np.pi)*theta[j]
	for j in range(len(psi)):
		psi[j] = (180./np.pi)*psi[j]

	f1 = load('f1','resultsADRC')
	f2 = load('f2','resultsADRC')
	f3 = load('f3','resultsADRC')
	f4 = load('f4','resultsADRC')
	f5 = load('f5','resultsADRC')
	f6 = load('f6','resultsADRC')

	u1 = load('u1','resultsADRC')
	u2 = load('u2','resultsADRC')
	u3 = load('u3','resultsADRC')
	u4 = load('u4','resultsADRC')
	u5 = load('u5','resultsADRC')
	u6 = load('u6','resultsADRC')
	
	plt.figure(1)
	plt.suptitle('Pose Regulation Results',fontsize=22)
	plt.subplot(3,2,1)
	plt.plot(x,'k',linewidth=3)
	plt.plot(xd,color='r',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel('x [m]',fontsize=18)

	plt.subplot(3,2,2)
	plt.plot(phi,'k',linewidth=3)
	plt.plot(phid,color='r',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\phi~[^o]$',fontsize=18)

	plt.subplot(3,2,3)
	plt.plot(y,'k',linewidth=3)
	plt.plot(yd,color='r',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel('y [m]',fontsize=18)

	plt.subplot(3,2,4)
	plt.plot(theta,'k',linewidth=3)
	plt.plot(thetad,color='r',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\theta~[^o]$',fontsize=18)

	plt.subplot(3,2,5)
	plt.plot(z,'k',linewidth=3)
	plt.plot(zd,color='r',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel('z [m]',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

	plt.subplot(3,2,6)
	plt.plot(psi,'k',linewidth=3)
	plt.plot(psid,color='r',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\psi~[^o]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)
#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
	plt.figure(2)
	plt.suptitle('Control signals',fontsize=22)
	plt.subplot(3,2,1)
	plt.plot(u1,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$f_x~[N]$',fontsize=18)

	plt.subplot(3,2,2)
	plt.plot(u4,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\tau_\phi~[Nm]$',fontsize=18)

	plt.subplot(3,2,3)
	plt.plot(u2,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$f_y~[N]$',fontsize=18)

	plt.subplot(3,2,4)
	plt.plot(u5,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\tau_\theta~[Nm]$',fontsize=18)

	plt.subplot(3,2,5)
	plt.plot(u3,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$f_z~[N]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

	plt.subplot(3,2,6)
	plt.plot(u6,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\tau_\psi~[Nm]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
	
	ex = np.zeros(len(x))
	ey = np.zeros(len(x))
	ez = np.zeros(len(x))
	ephi = np.zeros(len(x))
	etheta = np.zeros(len(x))
	epsi = np.zeros(len(x))
	
	for j in range(len(x)):
		ex[j] = xd[j]-x[j]
		ey[j] = yd[j]-y[j]
		ez[j] = zd[j]-z[j]
		ephi[j] = (180./np.pi)*( phid[j]-phi[j] )
		etheta[j] = (180./np.pi)*( thetad[j]-theta[j] )
		epsi[j] = (180./np.pi)*( psid[j]-psi[j] )
	
	plt.figure(3)
	plt.suptitle('Tracking Errors',fontsize=22)
	plt.subplot(3,2,1)
	plt.plot(ex,'k',linewidth=3)
	plt.grid()
	plt.ylabel('x [m]',fontsize=18)

	plt.subplot(3,2,2)
	plt.plot(ephi,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\phi~[^o]$',fontsize=18)

	plt.subplot(3,2,3)
	plt.plot(ey,'k',linewidth=3)
	plt.grid()
	plt.ylabel('y [m]',fontsize=18)

	plt.subplot(3,2,4)
	plt.plot(etheta,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\theta~[^o]$',fontsize=18)

	plt.subplot(3,2,5)
	plt.plot(ez,'k',linewidth=3)
	plt.grid()
	plt.ylabel('z [m]',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

	plt.subplot(3,2,6)
	plt.plot(epsi,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\psi~[^o]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)
#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
	plt.figure(4)
	plt.title('Thrust',fontsize=20)
	plt.plot(f1,color='#FF0000',linewidth=3)
	plt.plot(f2,color='#FF4000',linewidth=3)
	plt.plot(f3,color='#FF8000',linewidth=3)
	plt.plot(f4,color='#FFA000',linewidth=3)
	plt.plot(f5,color='#FFB400',linewidth=3)
	plt.plot(f6,color='#FFC800',linewidth=3)
	plt.legend(['f1','f2','f3','f4','f5','f6'],fontsize=18)
	plt.grid()
	plt.ylabel('Thrust [N]',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)
	
	plt.show()
#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
else:
	x = load2('x','resultsSMO')
	y = load2('y','resultsSMO')
	z = load2('z','resultsSMO')
	phi = load2('phi','resultsSMO')
	theta = load2('theta','resultsSMO')
	psi = load2('psi','resultsSMO')
	
	xg = load2('xg','resultsSMO')
	yg = load2('yg','resultsSMO')
	zg = load2('zg','resultsSMO')
	phig = load2('phig','resultsSMO')
	thetag = load2('thetag','resultsSMO')
	psig = load2('psig','resultsSMO')
	
	ddxg = load2('ddxg','resultsSMO')
	ddyg = load2('ddyg','resultsSMO')
	ddzg = load2('ddzg','resultsSMO')
	ddphig = load2('ddphig','resultsSMO')
	ddthetag = load2('ddthetag','resultsSMO')
	ddpsig = load2('ddpsig','resultsSMO')
		
	for i in phi:
		i = (180./np.pi)*i
	for i in theta:
		i = (180./np.pi)*i
	for i in psi:
		i = (180./np.pi)*i
	for i in phig:
		i = (180./np.pi)*i
	for i in thetag:
		i = (180./np.pi)*i
	for i in psig:
		i = (180./np.pi)*i
	for i in ddphig:
		i = (180./np.pi)*i
	for i in ddthetag:
		i = (180./np.pi)*i
	for i in ddpsig:
		i = (180./np.pi)*i
	
	plt.figure(1)
	plt.suptitle('Observer Tracking Results',fontsize=22)
	plt.subplot(3,2,1)
	plt.plot(x,'k',linewidth=3)
	plt.plot(xg,color='k',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel('x [m]',fontsize=18)

	plt.subplot(3,2,2)
	plt.plot(phi,'k',linewidth=3)
	plt.plot(phig,color='k',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\phi~[^o]$',fontsize=18)

	plt.subplot(3,2,3)
	plt.plot(y,'k',linewidth=3)
	plt.plot(yg,color='k',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel('y [m]',fontsize=18)

	plt.subplot(3,2,4)
	plt.plot(theta,'k',linewidth=3)
	plt.plot(thetag,color='k',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\theta~[^o]$',fontsize=18)

	plt.subplot(3,2,5)
	plt.plot(z,'k',linewidth=3)
	plt.plot(zg,color='k',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel('z [m]',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

	plt.subplot(3,2,6)
	plt.plot(psi,'k',linewidth=3)
	plt.plot(psig,color='k',linestyle='dashed',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\psi~[^o]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)
	
#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
	plt.figure(2)
	plt.suptitle('Disturbance Estimation Results',fontsize=22)
	plt.subplot(3,2,1)
	plt.plot(ddxg,'k',linewidth=3)
	plt.grid()
	plt.ylabel('x [N]',fontsize=18)

	plt.subplot(3,2,2)
	plt.plot(ddphig,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\phi~[NM]$',fontsize=18)

	plt.subplot(3,2,3)
	plt.plot(ddyg,'k',linewidth=3)
	plt.grid()
	plt.ylabel('y [N]',fontsize=18)

	plt.subplot(3,2,4)
	plt.plot(ddthetag,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\theta~[Nm]$',fontsize=18)

	plt.subplot(3,2,5)
	plt.plot(ddzg,'k',linewidth=3)
	plt.grid()
	plt.ylabel('z [N]',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

	plt.subplot(3,2,6)
	plt.plot(ddpsig,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\psi~[Nm]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)
	
#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
	exg = np.zeros(len(x))
	eyg = np.zeros(len(x))
	ezg = np.zeros(len(x))
	ephig = np.zeros(len(x))
	ethetag = np.zeros(len(x))
	epsig = np.zeros(len(x))
	
	for j in range(len(x)):
		exg[j] = x[j]-xg[j]
		eyg[j] = y[j]-yg[j]
		ezg[j] = z[j]-zg[j]
		ephig[j] = (180./np.pi)*( phi[j]-phig[j] )
		ethetag[j] = (180./np.pi)*( theta[j]-thetag[j] )
		epsig[j] = (180./np.pi)*( psi[j]-psig[j] )
	
	plt.figure(3)
	plt.suptitle('Observer Tracking Errors',fontsize=22)
	plt.subplot(3,2,1)
	plt.plot(exg,'k',linewidth=3)
	plt.grid()
	plt.ylabel('x [m]',fontsize=18)

	plt.subplot(3,2,2)
	plt.plot(ephig,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\phi~[^o]$',fontsize=18)

	plt.subplot(3,2,3)
	plt.plot(eyg,'k',linewidth=3)
	plt.grid()
	plt.ylabel('y [m]',fontsize=18)

	plt.subplot(3,2,4)
	plt.plot(ethetag,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\theta~[^o]$',fontsize=18)

	plt.subplot(3,2,5)
	plt.plot(ezg,'k',linewidth=3)
	plt.grid()
	plt.ylabel('z [m]',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)

	plt.subplot(3,2,6)
	plt.plot(epsig,'k',linewidth=3)
	plt.grid()
	plt.ylabel(r'$\psi~[^o]$',fontsize=18)
	plt.xlabel(r'$t~[s/10]$',fontsize=18)
	
	plt.show()
#----------------------------------------------------------------------
#----------------------------------------------------------------------
#----------------------------------------------------------------------
