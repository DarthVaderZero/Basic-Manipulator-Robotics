import matplotlib.pyplot as plt
m=1000
c=10
v=0
r=100
kp = 200
ki = 2.03125
kd = 50
velocity = []
time = []
e_old = r
e_i = 0
e_d = 0
t = 0
u = 0
for i in range(100):
    t=i
    e = r-v
    e_i+=e
    e_d = e - e_old
    u = kp*e + ki*e_i + kd*e_d
    v_new = ((u-c*v)/m)+v
    v = v_new
    velocity.append(v_new)
    time.append(t)
    #print(v_new)
    e_old = e
x1 = [0,100]
y1 = [100,100]
plt.plot(time,velocity)
plt.plot(x1,y1,label="Reference")
plt.show()
#kp = 200
#ki = 2.03125
#kd = 50