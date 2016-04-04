function R = getR(u, phi)
	I = eye(size(u, 1));
	R = I*cos(phi) + u*u'*(1-cos(phi)) + skew(u)*sin(phi);
end