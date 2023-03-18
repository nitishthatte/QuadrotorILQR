import unittest

from src.quadrotor_ilqr import main


class TestQuadrotorILQRMain(unittest.TestCase):
    def test_smoke_test(self):
        main(show_plots=False)


if __name__ == "__main__":
    unittest.main()
