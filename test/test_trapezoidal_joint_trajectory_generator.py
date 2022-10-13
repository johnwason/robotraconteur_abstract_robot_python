from robotraconteur_abstract_robot import trapezoidal_joint_trajectory_generator as trap_gen
from robotraconteur_abstract_robot.trapezoidal_joint_trajectory_generator \
    import TrapezoidalJointTrajectoryGeneratorCalc as trap_calc
from robotraconteur_abstract_robot.trapezoidal_joint_trajectory_generator \
    import TrapezoidalJointTrajectoryGeneratorExec as trap_exec
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

def _joint_limits():
    return trap_gen.JointTrajectoryLimits(
        j_max = np.array([0.43021,0.18482,0.90488]),
        a_max = np.array([0.97975,0.43887,0.11112]),
        v_max = np.array([0.25806,0.40872,0.5949]),
        x_min = np.array([-1.26221,-1.60284,-1.71122]),
        x_max = np.array([1.22175,1.11742,1.29668])
    )

def _pos_req1():
    return trap_gen.JointTrajectoryPositionRequest(
        current_position = np.array([0.31878,0.42417,-0.50786]),
        current_velocity = np.array([0.085516,-0.26248,0.80101]),
        desired_position = np.array([0.45885,0.96309,0.54681]),
        desired_velocity = np.array([0.02922,0.92885,0.73033]),
        max_velocity = np.array([0.25806,0.30872,0.5949]),
        desired_time = None,
        speed_ratio = 0.99,
        splice_time = None
    )
    
def _pos_req2():
    return trap_gen.JointTrajectoryPositionRequest(
        current_position = np.array([0.31878,0.42417,-0.50786]),
        current_velocity = np.array([0.,0.,0.]),
        desired_position = np.array([0.32,0.43,-0.508]),
        desired_velocity = np.array([0.,0.,0.]),
        max_velocity = np.array([0.25806,0.30872,0.5949]),
        desired_time = None,
        speed_ratio = 1,
        splice_time = None
    )

def _assert_pos_exec1(exec1):
    nptest.assert_allclose(exec1.joint_count, 3)
    nptest.assert_allclose(exec1.t1, 12.63499829104581)
    nptest.assert_allclose(exec1.t2, 1.9233462197923337)
    nptest.assert_allclose(exec1.t3, 11.992504236024757)
    nptest.assert_allclose(exec1.x1, [0.31878,0.42417,-0.50786])
    nptest.assert_allclose(exec1.v1, [0.085516,-0.26248,0.80101])
    nptest.assert_allclose(exec1.v2, [-0.04041468034412686,-0.236879856167895,-0.588951])
    nptest.assert_allclose(exec1.v3, [0.02922,0.92885,0.73033])
    nptest.assert_allclose(exec1.a1, [-0.009966814196830688,0.0020261295840655036,-0.11000879999999999])
    nptest.assert_allclose(exec1.a3, [0.005806517052122316,0.0972048734130202,0.11000879999999999])
    nptest.assert_allclose(exec1.xf, [0.45885,0.96309,0.54681])

def _assert_pos_exec2(exec2):
    nptest.assert_allclose(exec2.joint_count, 3)
    nptest.assert_allclose(exec2.t1, 0.11525673960878235)
    nptest.assert_allclose(exec2.t2, 0.)
    nptest.assert_allclose(exec2.t3, 0.11525673960878235)
    nptest.assert_allclose(exec2.x1, [0.31878,0.42417,-0.50786])
    nptest.assert_allclose(exec2.v1, [0.,0.,0.])
    nptest.assert_allclose(exec2.v2, [0.010585064302018804,0.050582725312106315,-0.0012146795100679844])
    nptest.assert_allclose(exec2.v3, [0,0,0])
    nptest.assert_allclose(exec2.a1, [0.09183900514579749,0.43887000000000004,-0.010538902229847807])
    nptest.assert_allclose(exec2.a3, [-0.09183900514579749,-0.43887000000000004,0.010538902229847807])
    nptest.assert_allclose(exec2.xf, [0.32,0.43,-0.508])
    


def test_trapezoidal_joint_trajectory_generator_calc_init_pos_exec():
    pos_exec1 = trap_calc.initialize_pos_exec(
        3, _joint_limits(), _pos_req1()
    )

    _assert_pos_exec1(pos_exec1)

    pos_exec2 = trap_calc.initialize_pos_exec(
        3, _joint_limits(), _pos_req2()
    )

    _assert_pos_exec2(pos_exec2)


def _vel_req1():
    return trap_gen.JointTrajectoryVelocityRequest(
        current_position = np.array([0.52114,0.23159,0.4889]),
        current_velocity = np.array([-0.2406,0.124,0.1429]),
        desired_velocity = np.array([0.153,0.308,0.4949]),
        timeout = 1.2,
        speed_ratio = 0.99
    )

def _assert_vel_exec1(exec2):
    nptest.assert_allclose(exec2.joint_count, 3)
    nptest.assert_allclose(exec2.t1, 1.2)
    nptest.assert_allclose(exec2.t2, 0.7942069328137202)
    nptest.assert_allclose(exec2.t3, 1.2)
    nptest.assert_allclose(exec2.x1, [0.52114,0.23159,0.4889])
    nptest.assert_allclose(exec2.v1, [-0.2406,0.124,0.1429])
    nptest.assert_allclose(exec2.v2, [0.153,0.308,0.13201055999999997])
    nptest.assert_allclose(exec2.v3, [0,0,0])
    nptest.assert_allclose(exec2.a1, [0.328,0.15333333333333335,-0.009074533333333357])
    nptest.assert_allclose(exec2.a3, [-0.1275,-0.2566666666666667,-0.11000879999999998])
    assert exec2.xf is None

def test_trapezoidal_joint_trajectory_generator_calc_init_vel_exec():

    vel_exec1 = trap_calc.initialize_vel_exec(
        3, _joint_limits(), _vel_req1()
    )

    _assert_vel_exec1(vel_exec1)

def test_trapezoidal_joint_trajectory_generator_exec_funcs():
    
    #pos_1
    nptest.assert_allclose(trap_exec.pos_1(0.0, 1.2, 1.3, 0.143), 1.4716)
    nptest.assert_allclose(trap_exec.pos_1(0.231, 2.8, 0.827, 0.928), 3.524866752)

    #vel_1
    nptest.assert_allclose(trap_exec.vel_1(0.7792, 0.9340, 0.1299), 1.03521808)

    #pos
    nptest.assert_allclose(trap_exec.pos(3,
        np.array([0.88517,0.91329,0.79618]),
        np.array([0.098712,0.26187,0.33536]),
        np.array([0.67973,0.13655,0.72123]),
        0.10676
    ), [0.695312943112296,0.169711942820552,0.7615703430375841])

    nptest.assert_allclose(trap_exec.pos(3,
        None,
        np.array([0.77905,0.71504,0.90372]),
        np.array([0.89092,0.33416,0.69875]),
        0.74407
    ),[1.4705877335,0.8661998128,1.3711809404])

def _assert_exec_calc_at_time(exec_, t, x, v):
    res, x_res, v_res = exec_.calc_at_time(t)
    assert res
    nptest.assert_allclose(x_res, x)
    nptest.assert_allclose(v_res, v)

def test_trapezoidal_joint_trajectory_generator_exec_calc_at_time1():

    pos_exec1 = trap_calc.initialize_pos_exec(
        3, _joint_limits(), _pos_req1()
    )
    
    _assert_exec_calc_at_time(pos_exec1, 0.0, [0.31878,0.42417,-0.50786], [0.085516,-0.26248,0.80101])
    _assert_exec_calc_at_time(pos_exec1, 0.1, [0.32728176592901587,0.3979321306479203,-0.42830904399999997], 
        [0.08451931858031693,-0.2622773870415934,0.79000912])
    _assert_exec_calc_at_time(pos_exec1, 12.1, [0.6239029667210094,-2.6035151837984847,1.1311667960000014], 
        [-0.03508245178165133,-0.2379638320328074,-0.5300964799999999])
    _assert_exec_calc_at_time(pos_exec1, 12.8, [0.5970390570643156,-2.769621045733643,0.7346446298101615], 
        [-0.04041468034412686,-0.236879856167895,-0.588951])
    _assert_exec_calc_at_time(pos_exec1,  14.2, [0.5404585045825381,-3.1012528443686955,-0.0898867701898376], 
        [-0.04041468034412686,-0.236879856167895,-0.588951])
    _assert_exec_calc_at_time(pos_exec1,  15.2, [0.5012391589681557,-3.3181220194263843,-0.6561912614416291], 
        [-0.03668889680472066,-0.17450781556914718,-0.5183632496238911])
    _assert_exec_calc_at_time(pos_exec1,  24.4, [0.40943311001054206,-0.8098836798235243,-0.7695607419814292], 
        [0.01673106007480464,0.7197770198306386,0.49371771037610857])
    _assert_exec_calc_at_time(pos_exec1,  27, [0.47197419961666603,1.3802841414763947,0.8748386347036179], 
        [0.02922,0.92885,0.73033])
    _assert_exec_calc_at_time(pos_exec1,  29, [0.530414199616666,3.2379841414763932,2.3354986347036166], 
        [0.02922,0.92885,0.73033])

def test_trapezoidal_joint_trajectory_generator_exec_calc_at_time2():

    pos_exec2 = trap_calc.initialize_pos_exec(
        3, _joint_limits(), _pos_req2()
    )
    
    _assert_exec_calc_at_time(pos_exec2, 0, [0.31878,0.42417,-0.50786], [0,0,0])
    _assert_exec_calc_at_time(pos_exec2, 0.105, [0.3192862625158662,0.426589270875,-0.507918095698542], 
        [0.009643095540308736,0.04608135,-0.0011065847341340197])
    _assert_exec_calc_at_time(pos_exec2, 0.125, [0.31948877384780316,0.42755700945302655,-0.5079413347038463], 
        [0.009690252960812922,0.046306700624212625,-0.0011119962414049929])
    _assert_exec_calc_at_time(pos_exec2, 0.21, [0.31998067694338306,0.42990766113108464,-0.5079977826000603], 
        [0.0018839375234201358,0.00900275062421263,-0.0002161895518679295])
    _assert_exec_calc_at_time(pos_exec2, 0.3, [0.32,0.43,-0.508], [0,0,0])
    _assert_exec_calc_at_time(pos_exec2, 0.5, [0.32,0.43,-0.508], [0,0,0])
    
def test_trapezoidal_joint_trajectory_generator_exec_calc_at_time2():

    vel_exec1 = trap_calc.initialize_vel_exec(
        3, _joint_limits(), _vel_req1()
    )
    
    _assert_exec_calc_at_time(vel_exec1, 0, [0.52114,0.23159,0.4889], [-0.2406,0.124,0.1429])
    _assert_exec_calc_at_time(vel_exec1, 1, [0.44454000000000005,0.4322566666666666,0.6272627333333334], 
        [0.0874,0.2773333333333333,0.13382546666666664])
    _assert_exec_calc_at_time(vel_exec1, 1.3, [0.48388000000000014,0.52159,0.667047392], 
        [0.153,0.308,0.13201055999999997])
    _assert_exec_calc_at_time(vel_exec1, 1.9, [0.5756800000000001,0.70639,0.7462537279999999], 
        [0.153,0.308,0.13201055999999997])
    _assert_exec_calc_at_time(vel_exec1, 2.1, [0.6055664989671267,0.7665536711233659,0.772040221235881], 
        [0.13951138393374932,0.28084644608885484,0.12037239163051794])
    _assert_exec_calc_at_time(vel_exec1, 3.2, [0.6818936607204993,0.9202057353066257,0.8378963739566214], [0,0,0])
    _assert_exec_calc_at_time(vel_exec1, 4, [0.6818936607204993,0.9202057353066257,0.8378963739566214], [0,0,0])


def _assert_gen_command_at_time(gen1, t, x, v):
    res, cmd = gen1.get_command(t)
    assert res
    nptest.assert_allclose(cmd.command_position, x)
    nptest.assert_allclose(cmd.command_velocity, v)


def test_trapezoidal_joint_trajectory_generator_exec1():

    gen1 = trap_gen.TrapezoidalJointTrajectoryGenerator(3, _joint_limits())

    gen1.update_desired_position(_pos_req1())

    _assert_gen_command_at_time(gen1, 0.0, [0.31878,0.42417,-0.50786], [0.085516,-0.26248,0.80101])
    _assert_gen_command_at_time(gen1, 0.1, [0.32728176592901587,0.3979321306479203,-0.42830904399999997], 
        [0.08451931858031693,-0.2622773870415934,0.79000912])
    _assert_gen_command_at_time(gen1, 12.1, [0.6239029667210094,-2.6035151837984847,1.1311667960000014], 
        [-0.03508245178165133,-0.2379638320328074,-0.5300964799999999])
    _assert_gen_command_at_time(gen1, 12.8, [0.5970390570643156,-2.769621045733643,0.7346446298101615], 
        [-0.04041468034412686,-0.236879856167895,-0.588951])
    _assert_gen_command_at_time(gen1,  14.2, [0.5404585045825381,-3.1012528443686955,-0.0898867701898376], 
        [-0.04041468034412686,-0.236879856167895,-0.588951])
    _assert_gen_command_at_time(gen1,  15.2, [0.5012391589681557,-3.3181220194263843,-0.6561912614416291], 
        [-0.03668889680472066,-0.17450781556914718,-0.5183632496238911])
    _assert_gen_command_at_time(gen1,  24.4, [0.40943311001054206,-0.8098836798235243,-0.7695607419814292], 
        [0.01673106007480464,0.7197770198306386,0.49371771037610857])
    _assert_gen_command_at_time(gen1,  27, [0.47197419961666603,1.3802841414763947,0.8748386347036179], 
        [0.02922,0.92885,0.73033])
    _assert_gen_command_at_time(gen1,  29, [0.530414199616666,3.2379841414763932,2.3354986347036166], 
        [0.02922,0.92885,0.73033])

def test_trapezoidal_joint_trajectory_generator_exec_calc_at_time2():

    gen2 = trap_gen.TrapezoidalJointTrajectoryGenerator(3, _joint_limits())

    gen2.update_desired_velocity(_vel_req1())
    
    _assert_gen_command_at_time(gen2, 0, [0.52114,0.23159,0.4889], [-0.2406,0.124,0.1429])
    _assert_gen_command_at_time(gen2, 1, [0.44454000000000005,0.4322566666666666,0.6272627333333334], 
        [0.0874,0.2773333333333333,0.13382546666666664])
    _assert_gen_command_at_time(gen2, 1.3, [0.48388000000000014,0.52159,0.667047392], 
        [0.153,0.308,0.13201055999999997])
    _assert_gen_command_at_time(gen2, 1.9, [0.5756800000000001,0.70639,0.7462537279999999], 
        [0.153,0.308,0.13201055999999997])
    _assert_gen_command_at_time(gen2, 2.1, [0.6055664989671267,0.7665536711233659,0.772040221235881], 
        [0.13951138393374932,0.28084644608885484,0.12037239163051794])
    _assert_gen_command_at_time(gen2, 3.2, [0.6818936607204993,0.9202057353066257,0.8378963739566214], [0,0,0])
    _assert_gen_command_at_time(gen2, 4, [0.6818936607204993,0.9202057353066257,0.8378963739566214], [0,0,0])
