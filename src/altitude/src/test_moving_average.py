import unittest 

from random import seed
from random import random

import numpy as np

from moving_average import MovingAverage


class TestMovingAverage(unittest.TestCase):

    def setUp(self):
        pass
 
    def test_empty_window(self):
        mw = MovingAverage(10)
        self.assertEqual(mw.get_average(), 0.0)

    def test_short_window(self):
        mw = MovingAverage(10)
        values = [random() for i in range(5)]
        for v in values:
            mw.add(v)
        self.assertTrue( abs(mw.get_average() - np.average(values) <= 0.0001 ) )

    def test_window(self):
        mw = MovingAverage(5)
        values = [random() for i in range(10)]
        for v in values:
            mw.add(v)
        self.assertTrue( abs(mw.get_average() - np.average(values[5:]) <= 0.0001 ) ) 