# Check absolute voltages

def gain_check(AVSS, AVDD, Vain_P, Vain_N, gain):
	Vin = Vain_P - Vain_N
	a = AVSS + 0.3 + Vin*(gain - 1)/2
	b = AVDD - 0.3 - Vin*(gain - 1)/2
	#~ print("\n", gain)
	#~ print("a", a, "Vain_P", Vain_P)
	#~ print("b", b, "Vain_N", Vain_N)
	if a < Vain_P and Vain_N < b:
		#~ print(gain, "is ok")
		pass
	elif a > Vain_P:
		print(gain, "a is bad")
		if b < Vain_N:
			print(gain, "b is bad")
	else:
		print(gain, "is bad")
		
def picture_check(AVSS, AVDD, Vain_P, Vain_N, gain):
	Vin = Vain_P - Vain_N
	
	a = Vain_P + Vin*(gain - 1)/2
	b = Vain_N - Vin*(gain - 1)/2
	

	
	if a < (AVDD - 0.3) and b > (AVSS + 0.3):
		#~ print(gain, "is ok")
		pass
	elif a > (AVDD - 0.3):
		print("\n", gain)
		print("Vain_P", Vain_P, "Vain_N", Vain_N, "Vin", Vin)
		print("a", a, "AVDD - 0.3", AVDD - 0.3)
		print("b", b, "AVSS + 0.3", AVSS + 0.3)
		print(gain, "a is bad")
		if b < (AVSS + 0.3):
			print(gain, "b is bad")
	elif b < (AVSS + 0.3):
		print("\n", gain)
		print("a", a, "AVDD - 0.3", AVDD - 0.3)
		print("b", b, "AVSS + 0.3", AVSS + 0.3)
		print(gain, "is bad")
		
differential_voltages = [114.48,122.29,224.53,89.77,52.68,95.86*100e-6*1.5]
differential_voltages[:] = [x/1000 for x in differential_voltages]

absolute_voltages = []
I = 100e-6
R = 20e3
Vref = I*R
for i in range(len(differential_voltages)):
	absolute_voltages.append(Vref+sum(differential_voltages[i:]))

#~ print(differential_voltages)
#~ print(absolute_voltages)
#~ import sys
#~ sys.exit()

AVSS, AVDD = 0, 5 # max AVSS and min AVDD voltages
for i in range(len(absolute_voltages)-1):
	Vain_P, Vain_N = absolute_voltages[i], absolute_voltages[i+1] # Absolute input voltage on + and - sides
	for gain in [1,2,4,8,16,32,64,128]:
		gain_check(AVSS, AVDD, Vain_P, Vain_N, gain)
		#~ picture_check(AVSS, AVDD, Vain_P, Vain_N, gain)

