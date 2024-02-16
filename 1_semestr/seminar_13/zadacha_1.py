import unittest
from dop_1 import main

class Test_defenititon(unittest.TestCase):

    def test_numberr(self):
        self.assertEqual(main(12), [2, 2, 3])
        self.assertEqual(main(0), [0])
        self.assertEqual(main(1),[1])
        self.assertEqual(main(1231231231),[257, 1049, 4567])
    def test_value(self):
        self.assertRaises(TypeError, main,'abs' )
        self.assertRaises(TypeError,main, True)
        self.assertRaises(TypeError, main, [1,12,34])
        self.assertRaises(ValueError, main, 0.5)