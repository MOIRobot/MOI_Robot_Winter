def CrossPoint():
	x1=1
	y1=0
	x2=1
	y2=1
	
	x3=0
	y3=0
	x4=-1
	y4=1
	if (x2==x1):
		k2=float(y4-y3)/(x4-x3)
		c2=y3-k2*x3
		#y=k2*x+c2
		Y=k2*x1+c2
		return x1,Y
	if (x3==x4):
		k1=float(y2-y1)/(x2-x1)
		c1=y1-k1*x1
		Y=k1*x3+c1
		return x3,Y
	k2=float(y4-y3)/(x4-x3)
	c2=y3-k2*x3
	k1=float(y2-y1)/(x2-x1)
	c1=y1-k1*x1
	X=float(c1-c2)/(k2-k1)
	Y=float(k1*c2-c1*k2)/(k1-k2)
	return X,Y
print CrossPoint()
