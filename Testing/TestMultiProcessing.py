"""import concurrent.futures
import math
from functools import partial
import numpy as np

PRIMES = [
    112272535095293,
    112582705942171,
    112272535095293,
    115280095190773,
    115797848077099,
    1099726899285419
]

def is_prime(n, test = np.arange(5)):
    print(test)
    if n < 2:
        return False
    if n == 2:
        return True
    if n % 2 == 0:
        return False

    sqrt_n = int(math.floor(math.sqrt(n)))
    for i in range(3, sqrt_n + 1, 2):
        if n % i == 0:
            return False
    return True

def main():
    with concurrent.futures.ProcessPoolExecutor() as executor:
        for number in PRIMES:
            prime = executor.submit(is_prime, number)
            print('%d is prime: %s' % (number, prime.result()))

def main():
    A = np.arange(5)
    f = partial(is_prime, test=A)
    with concurrent.futures.ProcessPoolExecutor() as executor:
        i=0
        for prime in executor.map(is_prime, PRIMES):
            number = PRIMES[i]
            print('%d is prime: %s' % (number, prime))
            i+=1

if __name__ == '__main__':
    main()"""

from concurrent.futures import ThreadPoolExecutor, as_completed

def add_one(number):
    return number + 1

def process():
    all_numbers = []
    for i in range(0, 10):
        all_numbers.append(i)

    all_results = []
    with ThreadPoolExecutor(max_workers=10) as executor:
        for i in executor.map(add_one, all_numbers):
            print(i)
            all_results.append(i)

    for index, result in enumerate(all_results):
        print(result)

process()