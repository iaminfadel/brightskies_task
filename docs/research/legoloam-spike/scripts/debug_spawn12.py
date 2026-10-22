import carla, time, math
c=carla.Client('127.0.0.1',2000); c.set_timeout(30)
w=c.get_world(); hero=w.get_actor(82); m=w.get_map()
sps=list(m.get_spawn_points())
for idx in (12,2,4,5):
    sp=sps[idx]
    print('test spawn',idx,sp.location,sp.rotation.yaw)
    hero.set_autopilot(False, 8000)
    hero.apply_control(carla.VehicleControl())
    hero.set_transform(sp)
    time.sleep(1.0)
    tm=c.get_trafficmanager(8000); tm.set_synchronous_mode(True); tm.set_random_device_seed(1234)
    hero.set_autopilot(True, 8000)
    tm.vehicle_percentage_speed_difference(hero, -20.0)
    for i in range(10):
        time.sleep(0.5)
        v=hero.get_velocity(); spd=math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)*3.6
        print('  ',i,round(spd,1),hero.get_transform().location)
        if spd>5: break
