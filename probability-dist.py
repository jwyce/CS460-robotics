import random
import decimal
import math
import numpy as np
import matplotlib.pyplot as plt


def float_range(start, stop, step):
    while start <= stop:
        yield float(start)
        start += decimal.Decimal(step)


def f(x):
    return .5 + (np.sign(x)/2)*math.sqrt(1-math.pow(math.e, -2*x**2 / math.pi))


def closest(y):
    closest = -float("inf")
    closest_delta = float("inf")

    for x in list(float_range(-5, 5, '0.2')):
        val = f(x)
        delta = abs(val - y)
        if delta < closest_delta:
            closest_delta = delta
            closest = x

    return closest


print(closest(0))
print(f(-5))

N = 500
data = []
for i in range(N):
    r = random.random()
    data.append(closest(r))

plt.hist(data, bins=50, range=(-5, 6))
plt.xticks(np.arange(-5, 6, 1))
plt.xlabel('Value of Sample')
plt.ylabel('Number of Samples')
plt.title('Histogram (N = 500)')
plt.show()
