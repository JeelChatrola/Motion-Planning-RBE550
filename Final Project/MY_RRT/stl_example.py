from STLFormula import *

alpha,beta,gamma,delta,t1,t2 = 150,200,300,350,20,40

test_pepper_condition = STLPredicate2D(0,1,alpha,beta,gamma,delta)
print(test_pepper_condition)

test_eventually_untimed = Untimed_Eventually(test_pepper_condition)
print(test_eventually_untimed)

test_eventually_timed = Eventually(test_pepper_condition,t1,t2)
print(test_eventually_untimed)

test_always_untimed = Untimed_Always(test_pepper_condition)
print(test_always_untimed)

test_always_timed = Always(test_pepper_condition,t1,t2)
print(test_always_untimed)

test_several_zones = Conjunction([Always(STLPredicate2D(0,1,400,450,50,100),15,25),Always(STLPredicate2D(0,1,100,150,350,400),35,55)])
print(test_always_untimed)