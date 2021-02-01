import numpy as np
import matplotlib.pyplot as plt

def load(arg,_file):
	if arg   == 'x':     k=0
	elif arg == 'y':     k=1
	elif arg == 'z':     k=2
	elif arg == 'phi':   k=3
	elif arg == 'theta': k=4
	elif arg == 'psi':   k=5
	elif arg == 'f1':    k=6
	elif arg == 'f2':    k=7
	elif arg == 'f3':    k=8
	elif arg == 'f4':    k=9
	elif arg == 'f5':    k=10
	elif arg == 'f6':    k=11
	elif arg == 'fz':    k=12
	else:
		print 'invalid argument'
		return 0
	col = []
	with open(_file+'.txt','r') as files:
		data = files.readlines()
		for line in data:
			col.append(float(line.rsplit(',')[k]))
	return col

x_py = load('x','resultsSMC')
y_py = load('y','resultsSMC')
z_py = load('z','resultsSMC')
phi_py = load('phi','resultsSMC')
the_py = load('theta','resultsSMC')
psi_py = load('psi','resultsSMC')

'''
x_c = load('x','resultsGFz')
y_c = load('y','resultsGFz')
z_c = load('z','resultsGFz')
phi_c = load('phi','resultsGFz')
the_c = load('theta','resultsGFz')
psi_c = load('psi','resultsGFz')
'''
f1_py = load('f1','resultsSMC')
f2_py = load('f2','resultsSMC')
f3_py = load('f3','resultsSMC')
f4_py = load('f4','resultsSMC')
f5_py = load('f5','resultsSMC')
f6_py = load('f6','resultsSMC')

'''
f1_c = load('f1','resultsGFz')
f2_c = load('f2','resultsGFz')
f3_c = load('f3','resultsGFz')
f4_c = load('f4','resultsGFz')
f5_c = load('f5','resultsGFz')
f6_c = load('f6','resultsGFz')

fz_py = load('fz','resultsX1m')
fz_c  = load('fz','resultsGFz')
'''
plt.figure(1)
plt.suptitle('Pose Regulation Results',fontsize=22)
plt.subplot(3,2,1)
plt.plot(x_py,color='#FF0000',linewidth=3)
#plt.plot(x_c,color='#FFA000',linewidth=3)
plt.plot(1.0*np.ones(len(x_py)),'k--',linewidth=3)
#plt.legend(['py','gaz','ref'],fontsize=18)
plt.grid()
plt.ylabel('x [m]',fontsize=18)
plt.subplot(3,2,2)
plt.plot(phi_py,color='#FF0000',linewidth=3)
#plt.plot(phi_c,color='#FFA000',linewidth=3)
plt.plot(0.0*np.ones(len(x_py)),'k--',linewidth=3)
plt.grid()
plt.ylabel('phi [rad]',fontsize=18)
plt.subplot(3,2,3)
plt.plot(y_py,color='#FF0000',linewidth=3)
#plt.plot(y_c,color='#FFA000',linewidth=3)
plt.plot(-1.0*np.ones(len(x_py)),'k--',linewidth=3)
plt.grid()
plt.ylabel('y [m]',fontsize=18)
plt.subplot(3,2,4)
plt.plot(the_py,color='#FF0000',linewidth=3)
#plt.plot(the_c,color='#FFA000',linewidth=3)
plt.plot(0.0*np.ones(len(x_py)),'k--',linewidth=3)
plt.grid()
plt.ylabel('theta [rad]',fontsize=18)
plt.subplot(3,2,5)
plt.plot(z_py,color='#FF0000',linewidth=3)
#plt.plot(z_c,color='#FFA000',linewidth=3)
plt.plot(2*np.ones(len(x_py)),'k--',linewidth=3)
plt.grid()
plt.ylabel('z [m]',fontsize=18)
plt.subplot(3,2,6)
plt.plot(psi_py,color='#FF0000',linewidth=3)
#plt.plot(psi_c,color='#FFA000',linewidth=3)
plt.plot(-(np.pi*20/180)*np.ones(len(x_py)),'k--',linewidth=3)
plt.grid()
plt.ylabel('psi [rad]',fontsize=18)

plt.figure(2)
plt.title('Thrust',fontsize=20)
plt.plot(f1_py,color='#FF0000',linewidth=3)
plt.plot(f2_py,color='#FF4000',linewidth=3)
plt.plot(f3_py,color='#FF8000',linewidth=3)
plt.plot(f4_py,color='#FFA000',linewidth=3)
plt.plot(f5_py,color='#FFB400',linewidth=3)
plt.plot(f6_py,color='#FFC800',linewidth=3)
plt.legend(['f1','f2','f3','f4','f5','f6'],fontsize=18)
plt.grid()
plt.ylabel('thrust [Nm]',fontsize=18)
'''
plt.title('Gazebo',fontsize=18)
plt.plot(f1_c,color='#FF0000',linewidth=3)
plt.plot(f2_c,color='#FF4000',linewidth=3)
plt.plot(f3_c,color='#FF8000',linewidth=3)
plt.plot(f4_c,color='#FFA000',linewidth=3)
plt.plot(f5_c,color='#FFB400',linewidth=3)
plt.plot(f6_c,color='#FFC800',linewidth=3)
plt.legend(['f1','f2','f3','f4','f5','f6'],fontsize=18)
plt.grid()
plt.ylabel('thrust [Nm]',fontsize=18)

plt.figure(3)

plt.title('Control Signal',fontsize=20)
plt.plot(fz_py,color='#FF0000',linewidth=3)
#plt.plot(fz_c,color='#FFA000',linewidth=3)
#plt.legend(['PY','GAZ'],fontsize=18)
plt.grid()
plt.ylabel('force in Z [N]',fontsize=18)
'''
plt.show()
