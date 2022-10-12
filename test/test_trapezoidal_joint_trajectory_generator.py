from robotraconteur_abstract_robot import trapezoidal_joint_trajectory_generator as trap_gen
from robotraconteur_abstract_robot.trapezoidal_joint_trajectory_generator \
    import TrapezoidalJointTrajectoryGeneratorCalc as trap_calc
import numpy as np
import numpy.testing as nptest

def test_trapezoidal_joint_trajectory_generator_calc_cases():

    #pos_1
    nptest.assert_allclose(trap_calc.pos_1(0.0, 1.2, 1.3, 0.143), 1.4716)
    nptest.assert_allclose(trap_calc.pos_1(0.231, 2.8, 0.827, 0.928), 3.524866752)

    #vel_1
    nptest.assert_allclose(trap_calc.vel_1(0.7792, 0.9340, 0.1299), 1.03521808)

    #pos
    nptest.assert_allclose(trap_calc.pos(0.9502,0.0344,0.4387,0.3815,0.7655,0.7951,0.1868), (2.098549805093, 1.1153040200000002))

    #solve_case2_sub
    nptest.assert_allclose(trap_calc.solve_case2_sub(0.7537, 0.3804, 0.5678, 0.0759, 0.0540, 0.5308),
        (False, -0.5308, -0.5308, 0.9267143933685003, -8.883303895082667, 0.04125847776940466))

    nptest.assert_allclose(trap_calc.solve_case2_sub(0.7537, 0.3804, 0.5678, 0.5678, 0.0540, 0.5308),
        (False, 0, -0.5308, 0, -1.1874652441648021, 0.9679728711379049))

    nptest.assert_allclose(trap_calc.solve_case2_sub(0.7537, 0.3804, 0.5678, 0.0759, 0.0759, 0.5308),
        (False, -0.5308, 0, 0.9267143933685003, -8.84799772734719, 0))

    #solve_case2
    nptest.assert_allclose(trap_calc.solve_case2(0.77491,0.8173,0.86869,0.084436,0.39978,0.25987), 
        (True, -0.39978, -0.25987, 0.25987, 4.881171354908224, 2.021698567894567, 1.863300881209836))
    nptest.assert_allclose(trap_calc.solve_case2(0.13607,0.86929,0.5797,0.54986,0.14495,0.85303),
        (True, 0.14495, -0.85303, 0.85303, 0.5096538222571304, 2.646816358788585, 0.47467263753912525))
    assert trap_calc.solve_case2(0.13607,0.137,0,0,0.14495,0.85303) == \
        (False, None, None, None, None, None, None)

    #solve_case3_sub1
    nptest.assert_allclose(trap_calc.solve_case3_sub1(0.62206,0.35095,0.51325,0.40181,0.075967), 0.19184300593)

    #solve_case3
    nptest.assert_allclose(trap_calc.solve_case3(0.23992,0.12332,0.18391,0.23995,0.41727), 
        (True, -0.41727, 0.41727, 1.1768867678518975, 1.3111882992344557))
    nptest.assert_allclose(trap_calc.solve_case3(0.049654,0.90272,0.94479,0.49086,0.48925),
        (True, 0.48925, -0.48925, 0.09658156466979077, 1.0243894338573225))
    nptest.assert_allclose(trap_calc.solve_case3(0.18974,0.24169,-0.40391,-0.096455,0.13197),
        (True, 0.13197, -0.13197, 5.372425192883086, 3.0426911624216175))
    nptest.assert_allclose(trap_calc.solve_case3(0.049654,2.90272,0.0,0.0,0.48925),
        (True, 0.48925, -0.48925, 2.414851849129623, 2.414851849129623))

    #solve_case4
    nptest.assert_allclose(trap_calc.solve_case4(0.64912,0.73172,0.64775,0.45092,0.54701,0.29632,0.74469),
        (-0.2785703972213084, -1.6934249780101067, 0.9795893555993882))
    nptest.assert_allclose(trap_calc.solve_case4(0.18896,0.68678,0.18351,0.36848,0.62562,0.78023,0.0),
        (0.40292783146087974, 0.35072061548684463, 0.0))

    #solve_case5
    nptest.assert_allclose(trap_calc.solve_case5(0.92939,0.77571,0.48679,0.43586,0.44678), 
        (True, 0.77571, -0.44678, -0.44678, 0.3439724249071132, 0.09188757509288681, 0.646671740006267))
    nptest.assert_allclose(trap_calc.solve_case5(0.30635,0.30635,0.51077,0.81763,0.79483), 
        (True, 0.30635, 0, 0.79483, 0, 0.81763, 0.25718707144924063))
    nptest.assert_allclose(trap_calc.solve_case5(0.64432,0.37861,0.81158,0.53283,0.35073), 
        (True, -0.1868794659, -0.35073, 0.35073, 0.53283, 0.0, 2.8468037119721723))
    nptest.assert_allclose(trap_calc.solve_case5(0.939,0.87594,0.87594,0.62248,0.58704), 
        (True, 0.87594, -0.58704, 0, 0.10742027800490579, 0.5150597219950942, 0.0))    

    #solve_case6
    nptest.assert_allclose(trap_calc.solve_case6(0.20774,0.30125,0.47092,0.23049,0.84431,0.19476),
        (0.4057008980866849, 0.8711747792154446))
    nptest.assert_allclose(trap_calc.solve_case6(0.22592,0.17071,0.22766,0.4357,0.3111,0.0),
        (-0.12671563002065644, 0))

    
    