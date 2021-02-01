#!/usr/bin/python2.7
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
import time

xd = 0.0#; yd = -2.0; zd = 2.0;
#phid=0.0*np.pi/180.0; thed=0.0*np.pi/180.0; psid= 0.0*np.pi/180.0;


def callback(data):
	print 'roar'
	print data.data
	if not data.data>0:
		x = data.data[0]
		y = data.data[1]
		z = data.data[2]
		phi = data.data[3]
		the = data.data[4]
		psi = data.data[5]
		t = time.time()
		plt.suptitle('Error signals',fontsize=20)
		plt.subplot(3,2,1)
		#plt.plot(t,xd,color='#ff0000',marker='o')
		plt.plot(t,x ,color='#000000',marker='o')
		#plt.ylabel('$e_x$')
		plt.subplot(3,2,3)
		#plt.plot(t,xd,color='#ff0000',marker='o')
		plt.plot(t,y ,color='#000000',marker='o')
		#plt.ylabel('$e_y$')
		plt.subplot(3,2,5)
		#plt.plot(t,xd,color='#ff0000',marker='o')
		plt.plot(t,z ,color='#000000',marker='o')
		#plt.ylabel('$e_z$')
		plt.subplot(3,2,2)
		#plt.plot(t,xd ,color='#ff0000',marker='o')
		plt.plot(t,phi,color='#000000',marker='o')
		#plt.ylabel('$e_{\phi}$')
		plt.subplot(3,2,4)
		#plt.plot(t,xd ,color='#ff0000',marker='o')
		plt.plot(t,the,color='#000000',marker='o')
		#plt.ylabel('$e_{ \theta}$')
		plt.subplot(3,2,6)
		#plt.plot(t,xd ,color='#ff0000',marker='o')
		plt.plot(t,psi,color='#000000',marker='o')
		#plt.ylabel('$e_{\psi}$')
		#plt.axis('equal')
		plt.draw()
		plt.pause(0.000000001)
	else:
		print 'no message'
	
def plotter():
	rospy.init_node("data_plotter",anonymous=True)
	rospy.Subscriber("error_signals", Float32MultiArray, callback)
	plt.ion()
	plt.show()
	rospy.spin()

if __name__ == '__main__':
	plotter()

'''	
#-----------------------CONTROLLED STATES-------------------------------
plt.figure(1)
#plt.suptitle('Controlled states using ADRC',fontsize=22)
plt.subplot(3,2,1)
plt.plot(x,color='#000000',linewidth=3)
plt.plot(xd*np.ones(len(x)),'r--',linewidth=3)
#plt.plot(x1g,'k--',linewidth=3)
#plt.legend(['$x$','$\hat{x}$','$x^d$'],fontsize=16)
#plt.legend(['$x$','$x^d$'],fontsize=16)
plt.grid()
plt.ylabel('$x$ [m]',fontsize=18)

plt.subplot(3,2,3)
plt.plot(y,color='#000000',linewidth=3)
plt.plot(yd*np.ones(len(y)),'r--',linewidth=3)
#plt.plot(x2g,'k--',linewidth=3)
#plt.legend(['$y$','$\hat{y}$','$y^d$'],fontsize=16)
#plt.legend(['$y$','$y^d$'],fontsize=16)
plt.grid()
plt.ylabel('$y$ [m]',fontsize=18)

plt.subplot(3,2,5)
plt.plot(z,color='#000000',linewidth=3)
plt.plot(zd*np.ones(len(z)),'r--',linewidth=3)
#plt.plot(x3g,'k--',linewidth=3)
#plt.legend(['$z$','$\hat{z}$','$z^d$'],fontsize=16)
#plt.legend(['$z$','$z^d$'],fontsize=16)
plt.grid()
plt.ylabel('$z$ [m]',fontsize=18)

plt.subplot(3,2,2)
plt.plot(phi,color='#000000',linewidth=3)
plt.plot(phid*np.ones(len(phi)),'r--',linewidth=3)
#plt.plot(x4g,'k--',linewidth=3)
#plt.legend([r'$\phi$',r'$\hat{\phi}$',r'$\phi^d$'],fontsize=16,loc='lower right')
#plt.legend([r'$\phi$',r'$\phi^d$'],fontsize=16,loc='lower right')
plt.grid()
plt.ylabel(r'$\phi$ [rad]',fontsize=18)

plt.subplot(3,2,4)
plt.plot(the,color='#000000',linewidth=3)
plt.plot(thed*np.ones(len(the)),'r--',linewidth=3)
#plt.plot(x5g,'k--',linewidth=3)
#plt.legend([r'$\theta$',r'$\hat{\theta}$',r'$\theta^d$'],fontsize=16,loc='lower right')
#plt.legend([r'$\theta$',r'$\theta^d$'],fontsize=16,loc='lower right')
plt.grid()
plt.ylabel(r'$\theta$ [rad]',fontsize=18)

plt.subplot(3,2,6)
plt.plot(psi,color='#000000',linewidth=3)
plt.plot(psid*np.ones(len(psi)),'r--',linewidth=3)
#plt.plot(x6g,'k--',linewidth=3)
#plt.legend([r'$\psi$',r'$\hat{\psi}$',r'$\psi^d$'],fontsize=16,loc='lower right')
#plt.legend([r'$\psi$',r'$\psi^d$'],fontsize=16,loc='lower right')
plt.grid()
plt.ylabel(r'$\psi$ [rad]',fontsize=18)
#-------------------------ERROR SIGNALS-----------------------------------
plt.figure(2)
#plt.suptitle('Error signals with ADRC',fontsize=22)
plt.subplot(3,2,1)
plt.plot(xd*np.ones(len(x))-x[:],'k',linewidth=3)
plt.grid()
plt.ylabel('$x$ [m]',fontsize=18)

plt.subplot(3,2,3)
plt.plot(yd*np.ones(len(y))-y[:],'k',linewidth=3)
plt.grid()
plt.ylabel('$y$ [m]',fontsize=18)

plt.subplot(3,2,5)
plt.plot(zd*np.ones(len(z))-z[:],'k',linewidth=3)
plt.grid()
plt.ylabel('$z$ [m]',fontsize=18)

plt.subplot(3,2,2)
plt.plot(phid*np.ones(len(phi))-phi[:],'k',linewidth=3)
plt.grid()
plt.ylabel(r'$\phi$ [rad]',fontsize=18)

plt.subplot(3,2,4)
plt.plot(thed*np.ones(len(the))-the[:],'k',linewidth=3)
plt.grid()
plt.ylabel(r'$\theta$ [Nm]',fontsize=18)

plt.subplot(3,2,6)
plt.plot(psid*np.ones(len(psi))-psi[:],'k',linewidth=3)
plt.grid()
plt.ylabel(r'$\psi$ [Nm]',fontsize=18)

#--------------------CONTROL SIGNALS------------------------------
plt.figure(3)
#plt.suptitle('Control signals of the ADRC',fontsize=22)
plt.subplot(3,2,1)
plt.plot(fx,color='k',linewidth=3)
plt.grid()
plt.ylabel('$F_x$ [N]',fontsize=18)

plt.subplot(3,2,3)
plt.plot(fy,color='k',linewidth=3)
plt.grid()
plt.ylabel('$F_y$ [N]',fontsize=18)

plt.subplot(3,2,5)
plt.plot(fz,color='k',linewidth=3)
plt.grid()
plt.ylabel('$F_z$ [N]',fontsize=18)

plt.subplot(3,2,2)
plt.plot(tx,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\tau_x$ [Nm]',fontsize=18)

plt.subplot(3,2,4)
plt.plot(ty,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\tau_y$ [Nm]',fontsize=18)

plt.subplot(3,2,6)
plt.plot(tz,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\tau_z$ [Nm]',fontsize=18)
#-------------------------THRUST-----------------------------------
plt.figure(4)
#plt.title('Thrusts using ADRC',fontsize=22)
plt.plot(f1,color='#FF0000',linewidth=3)
plt.plot(f2,color='#FF4000',linewidth=3)
plt.plot(f3,color='#FF8000',linewidth=3)
plt.plot(f4,color='#FFA000',linewidth=3)
plt.plot(f5,color='#FFB400',linewidth=3)
plt.plot(f6,color='#FFC800',linewidth=3)
#plt.legend(['$f_1$','$f_2$','$f_3$','$f_4$','$f_5$','$f_6$'],fontsize=18)
plt.grid()
plt.ylabel('Thrust [N]',fontsize=18)
#-------------------------DISTURBANCES-----------------------------------
plt.figure(5)
#plt.suptitle('Disturbances computed by the robust ESO',fontsize=22)
plt.subplot(3,2,1)
plt.plot(x13g,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\delta_x$ [N]',fontsize=18)

plt.subplot(3,2,3)
plt.plot(x23g,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\delta_y$ [N]',fontsize=18)

plt.subplot(3,2,5)
plt.plot(x33g,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\delta_z$ [N]',fontsize=18)

plt.subplot(3,2,2)
plt.plot(x43g,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\delta_\phi$ [Nm]',fontsize=18)

plt.subplot(3,2,4)
plt.plot(x53g,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\delta_\theta$ [Nm]',fontsize=18)

plt.subplot(3,2,6)
plt.plot(x63g,color='k',linewidth=3)
plt.grid()
plt.ylabel(r'$\delta_\psi$ [Nm]',fontsize=18)

plt.show()
'''
