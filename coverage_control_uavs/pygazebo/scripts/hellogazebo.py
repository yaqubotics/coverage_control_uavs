#!/usr/bin/env python
import trollius
from trollius import From
import pygazebo
from msg import boxgeom_pb2
from msg import vector3d_pb2
from msg import visual_pb2
from msg import pose_pb2
from msg import quaternion_pb2
from msg import geometry_pb2
from msg import material_pb2
from msg import color_pb2
from msg import pointcloud_pb2
from msg import spheregeom_pb2
from msg import cylindergeom_pb2
from msg import collision_pb2
from msg import surface_pb2
from msg import friction_pb2
from msg import link_pb2
from msg import model_pb2
from msg import inertial_pb2
from msg import time_pb2
from msg import poses_stamped_pb2
from msg import model_v_pb2
def ps(list_of_points):
	lp = list_of_points
	new_list = []
	for i in range(len(list_of_points)):
		new_list.append(pygazebo.msg.vector3d_pb2.Vector3d(x=lp[i][0],y=lp[i][1],z=lp[i][2]))
	return new_list
def add_box(manager,pos,size,id):
    idx=id+20
    model_publisher = yield From(manager.advertise('/gazebo/default/model/info','gazebo.msgs.Model'))
    color = pygazebo.msg.color_pb2.Color(r=1,g=0,b=0,a=1)

    pose = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=pos[0],y=pos[1],z=pos[2]),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=pos[6],x=pos[3],y=pos[4],z=pos[5]))
    vectorsize = pygazebo.msg.vector3d_pb2.Vector3d(x=size[0],y=size[1],z=size[2])
    zeropose = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    lbox = pygazebo.msg.boxgeom_pb2.BoxGeom(size=vectorsize)
    geobox = pygazebo.msg.geometry_pb2.Geometry(type=1,box=lbox)
    vector111 = pygazebo.msg.vector3d_pb2.Vector3d(x=1,y=1,z=1)
    linertial = pygazebo.msg.inertial_pb2.Inertial(mass=1,pose=zeropose,ixx=0.167,ixy=0,ixz=0,iyy=0.167,iyz=0,izz=0.167)
    mat = pygazebo.msg.material_pb2.Material(shader_type=2,ambient=color,diffuse=color) # ,emissive=color)  ,specular=color 

    lvisuallink1 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1",parent_id=20,name="unit_box1::link",id=21,pose=zeropose)
    lvisuallink2 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1::link",parent_id=21,name="unit_box1::link::visual",id=22,pose=zeropose,geometry=geobox,material=mat,is_static=False)
    lvisualcollision1 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1::link::collision",parent_id=21,name="unit_box1::link",id=23,pose=zeropose)
    vector000 = pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0)
    lvisualcollision2 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1::link::collision__COLLISION_VISUAL__",parent_id=21,name="unit_box1::link::visual",id=24,pose=zeropose,geometry=geobox,material=mat,is_static=False,cast_shadows=False)
    lfriction = pygazebo.msg.friction_pb2.Friction(fdir1=vector000,mu=1,mu2=1,slip1=0,slip2=0)
   
    lsurface = pygazebo.msg.surface_pb2.Surface(friction=lfriction,restitution_coefficient=0,bounce_threshold=100000,soft_cfm=0,soft_erp=0.2,kp=1000000000000,kd=1,max_vel=0.01,min_depth=0,collide_without_contact=False,collide_without_contact_bitmask=1)

    lcoll = pygazebo.msg.collision_pb2.Collision(laser_retro=0,id=23,name="unit_box1::link::collision",pose=zeropose,geometry=geobox,surface=lsurface,visual=[lvisualcollision1,lvisualcollision2])
    
    llink = pygazebo.msg.link_pb2.Link(inertial=linertial,canonical=True,collision=[lcoll],enabled=True,gravity=False,id=22,kinematic=False,name="unit_box1::link",pose=zeropose,self_collide=False,visual=[lvisuallink1,lvisuallink2]) # projector, sensor, inertial
    lvisualmodel = pygazebo.msg.visual_pb2.Visual(name="unit_box1",id=20,parent_name="default",parent_id=1,pose=pose)
    print "add_box"
    yield From(model_publisher.publish(pygazebo.msg.model_pb2.Model(name="unit_box1",id=20,is_static=False,pose=pose,link=[llink],visual=[lvisualmodel],scale=vector111)))


def main():
    manager = yield From(pygazebo.connect())
    '''
    string_publisher = yield From(manager.advertise('/gazebo/default/topic','gazebo.msgs.GzString'))
    #points_publisher = yield From(manager.advertise('/gazebo/default/point','gazebo.msgs.Visual'))
    #publisher = yield From(manager.advertise('gazebo/default/box','gazebo.msgs.BoxGeom'))
    model_publisher = yield From(manager.advertise('/gazebo/default/model/info','gazebo.msgs.Model'))

    publisher = yield From(manager.advertise('/gazebo/default/visual','gazebo.msgs.Visual'))
    collision_publisher = yield From(manager.advertise('/gazebo/default/collision','gazebo.msgs.Collision'))
    
    link_publisher = yield From(manager.advertise('/gazebo/default/link','gazebo.msgs.Link'))
    yield From(trollius.sleep(1))
    list_of_points = [(1,1,1),(2,2,2),(3,3,3)]
    #manager.start()

    #manager.subscribe('/gazebo/default/topic',
    #                  'gazebo.msgs.GzString',
    #                  callback)
    color = pygazebo.msg.color_pb2.Color(r=1,g=0,b=0,a=1)
    mat = pygazebo.msg.material_pb2.Material(shader_type=2,ambient=color,diffuse=color) # ,emissive=color)  ,specular=color 
    vector111 = pygazebo.msg.vector3d_pb2.Vector3d(x=1,y=1,z=1)
    vector2 = pygazebo.msg.vector3d_pb2.Vector3d(x=10,y=10,z=1)
    lcylinder = pygazebo.msg.cylindergeom_pb2.CylinderGeom(radius=2,length=5)
    lbox = pygazebo.msg.boxgeom_pb2.BoxGeom(size=vector111)
    lsphere = pygazebo.msg.spheregeom_pb2.SphereGeom(radius = 4)
    geo = pygazebo.msg.geometry_pb2.Geometry(type=1,box=lbox) 
    geosphere = pygazebo.msg.geometry_pb2.Geometry(type=3,sphere=lsphere)
    geocylinder = pygazebo.msg.geometry_pb2.Geometry(type=2,cylinder=lcylinder)
    geopoints = pygazebo.msg.geometry_pb2.Geometry(points=[pygazebo.msg.vector3d_pb2.Vector3d(x=1,y=1,z=1),pygazebo.msg.vector3d_pb2.Vector3d(x=2,y=1,z=1)])#ps(list_of_points))
    lpose = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=1,y=1,z=0),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    pose0 = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=3,y=3,z=3),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    zeropose = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    geobox = pygazebo.msg.geometry_pb2.Geometry(type=1,box=lbox)
    lpoints = pygazebo.msg.pointcloud_pb2.PointCloud(points=[pygazebo.msg.vector3d_pb2.Vector3d(x=10,y=10,z=0)])
    #yield From(publisher.publish(pygazebo.msg.visual_pb2.Visual(material=mat,cast_shadows=True,parent_name="ground_plane",name="abcdefg",id=1,scale=vector111,geometry=geocylinder,pose=pose0,visible=True,is_static=True)))
    lvisuallink1 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1",parent_id=20,name="unit_box1::link",id=21,pose=zeropose)
    lvisuallink2 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1::link",parent_id=21,name="unit_box1::link::visual",id=22,pose=zeropose,geometry=geobox,material=mat,is_static=False)
    lvisualcollision1 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1::link::collision",parent_id=21,name="unit_box1::link",id=23,pose=zeropose)
    lvisualcollision2 = pygazebo.msg.visual_pb2.Visual(parent_name="unit_box1::link::collision__COLLISION_VISUAL__",parent_id=21,name="unit_box1::link::visual",id=24,pose=zeropose,geometry=geobox,material=mat,is_static=False,cast_shadows=False)
    lvisualmodel = pygazebo.msg.visual_pb2.Visual(name="unit_box1",id=20,parent_name="default",parent_id=1,pose=pose0)
    vector000 = pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0)

    lfriction = pygazebo.msg.friction_pb2.Friction(fdir1=vector000,mu=1,mu2=1,slip1=0,slip2=0)
    lsurface = pygazebo.msg.surface_pb2.Surface(friction=lfriction,restitution_coefficient=0,bounce_threshold=100000,soft_cfm=0,soft_erp=0.2,kp=1000000000000,kd=1,max_vel=0.01,min_depth=0,collide_without_contact=False,collide_without_contact_bitmask=1)
    lcoll = pygazebo.msg.collision_pb2.Collision(laser_retro=0,id=23,name='unit_box1::link::collision',pose=zeropose,geometry=geobox,surface=lsurface,visual=[lvisualcollision1,lvisualcollision2])
    linertial = pygazebo.msg.inertial_pb2.Inertial(mass=1,pose=zeropose,ixx=0.167,ixy=0,ixz=0,iyy=0.167,iyz=0,izz=0.167)
    llink = pygazebo.msg.link_pb2.Link(inertial=linertial,canonical=True,collision=[lcoll],enabled=True,gravity=False,id=22,kinematic=False,name="unit_box1::link",pose=zeropose,self_collide=False,visual=[lvisuallink1,lvisuallink2]) # projector, sensor, inertial

    for i in xrange(1):
        yield From(string_publisher.publish(
            pygazebo.msg.gz_string_pb2.GzString(data='heloooo')))
        #yield From(publisher.publish(pygazebo.msg.boxgeom_pb2.BoxGeom(size=pygazebo.msg.vector3d_pb2.Vector3d(x=1,y=1,z=1))))
        #yield From(publisher.publish(pygazebo.msg.visual_pb2.Visual(material=mat,cast_shadows=True,parent_name="ground_plane",name="box1",id=1,scale=vector111,geometry=geo,pose=lpose,visible=True,is_static=True)))
        #yield From(points_publisher.publish(pygazebo.msg.visual_pb2.Visual(material=mat,cast_shadows=True,parent_name="ground_plane",name="abcdefg",id=1,scale=vector111,geometry=geosphere,pose=pose0,visible=True,is_static=True)))
        #yield From(publisher.publish(pygazebo.msg.visual_pb2.Visual(material=mat,cast_shadows=True,parent_name="ground_plane",name="abcdefg",id=1,scale=vector111,geometry=geocylinder,pose=pose0,visible=True,is_static=True)))
        #yield From(publisher.publish(pygazebo.msg.visual_pb2.Visual(material=mat,parent_name="ground_plane",name="abcdefg",id=1,geometry=geopoints,visible=True,is_static=True)))
        #yield From(collision_publisher.publish(pygazebo.msg.collision_pb2.Collision(id=1,name='col1',geometry=geocylinder,visual=[lvisual],surface=lsurface,pose=pose0,max_contacts=20.0)))
        #yield From(link_publisher.publish(pygazebo.msg.link_pb2.Link(canonical=True,collision=[lcoll],enabled=True,gravity=False,id=1,kinematic=False,name="link1",pose=pose0,self_collide=True,visual=[lvisual]))) # projector, sensor, inertial
        yield From(model_publisher.publish(pygazebo.msg.model_pb2.Model(name="unit_box1",id=20,is_static=False,pose=pose0,link=[llink],visual=[lvisualmodel],scale=vector111)))
        #yield add_box(manager,(0,0,0,0,0,0,1),(3,2,1),1)
        print "publish",i
        yield From(trollius.sleep(1))
    
    '''
    #string_publisher = yield From(manager.advertise('/gazebo/default/topic','gazebo.msgs.GzString'))
    model_publisher = yield From(manager.advertise('/gazebo/default/model/info','gazebo.msgs.Model'))
    model_v_publisher = yield From(manager.advertise('/gazebo/default/test/info','gazebo.msgs.Model_V'))
    visual_publisher = yield From(manager.advertise('/gazebo/default/visual','gazebo.msgs.Visual'))
    #collision_publisher = yield From(manager.advertise('/gazebo/default/collision','gazebo.msgs.Collision'))
    posestamped_publisher = yield From(manager.advertise('/gazebo/default/pose/info','gazebo.msgs.PosesStamped'))
    #link_publisher = yield From(manager.advertise('/gazebo/default/link','gazebo.msgs.Link'))
    yield From(trollius.sleep(1))
    color = pygazebo.msg.color_pb2.Color(r=1,g=0,b=0,a=1)
    pose0 = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=3,y=3,z=4),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    vector111 = pygazebo.msg.vector3d_pb2.Vector3d(x=1,y=1,z=1)

    #pose = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=2),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    vectorsize = pygazebo.msg.vector3d_pb2.Vector3d(x=2,y=2,z=8)
    zeropose = pygazebo.msg.pose_pb2.Pose(position=pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    lbox = pygazebo.msg.boxgeom_pb2.BoxGeom(size=vectorsize)
    geobox = pygazebo.msg.geometry_pb2.Geometry(type=1,box=lbox)
    linertial = pygazebo.msg.inertial_pb2.Inertial(mass=1,pose=zeropose,ixx=0.167,ixy=0,ixz=0,iyy=0.167,iyz=0,izz=0.167)
    mat = pygazebo.msg.material_pb2.Material(shader_type=2,ambient=color,diffuse=color) # ,emissive=color)  ,specular=color 
    vector000 = pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0)

    lvisuallink1 = pygazebo.msg.visual_pb2.Visual(name="unit_box1::link",id=151,parent_name="unit_box1",parent_id=150,pose=zeropose)
    lvisuallink2 = pygazebo.msg.visual_pb2.Visual(name="unit_box1::link::visual",id=152,parent_name="unit_box1::link",parent_id=151,geometry=geobox,material=mat,is_static=False)
    lvisualcollision1 = pygazebo.msg.visual_pb2.Visual(name="unit_box1::link::collision",id=153,parent_name="unit_box1::link",parent_id=151,pose=zeropose)
    lvisualcollision2 = pygazebo.msg.visual_pb2.Visual(name="unit_box1::link::collision__COLLISION_VISUAL__",id=154,parent_name="unit_box1::link::visual",parent_id=151,pose=zeropose,geometry=geobox,material=mat,is_static=False)
    lfriction = pygazebo.msg.friction_pb2.Friction(fdir1=vector000,mu=1,mu2=1,slip1=0,slip2=0) #torsional
   
    lsurface = pygazebo.msg.surface_pb2.Surface(friction=lfriction,restitution_coefficient=0,bounce_threshold=100000,soft_cfm=0,soft_erp=0.2,kp=1000000000000,kd=1,max_vel=0.01,min_depth=0,collide_without_contact=False,collide_without_contact_bitmask=1)

    lcoll = pygazebo.msg.collision_pb2.Collision(id=153,name="unit_box1::link::collision",laser_retro=0,pose=zeropose,geometry=geobox,surface=lsurface,visual=[lvisualcollision1,lvisualcollision2])
    
    llink = pygazebo.msg.link_pb2.Link(id=151,name="unit_box1::link",self_collide=False,gravity=False,kinematic=False,enabled=True,inertial=linertial,pose=zeropose,visual=[lvisuallink1,lvisuallink2],collision=[lcoll],canonical=True) # projector, sensor, inertial
    lvisualmodel = pygazebo.msg.visual_pb2.Visual(name="unit_box1",id=150,parent_name="default",parent_id=1,pose=pose0,type=1)
    #yield From(string_publisher.publish(
    #        pygazebo.msg.gz_string_pb2.GzString(data='heloooo')))
    ltime = pygazebo.msg.time_pb2.Time(sec=0,nsec=0)
    pose0 = pygazebo.msg.pose_pb2.Pose(name="unit_box1",id=150,position=pygazebo.msg.vector3d_pb2.Vector3d(x=3,y=3,z=4),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    zeropose = pygazebo.msg.pose_pb2.Pose(name="unit_box1::link",id=151,position=pygazebo.msg.vector3d_pb2.Vector3d(x=0,y=0,z=0),orientation=pygazebo.msg.quaternion_pb2.Quaternion(w=1,x=0,y=0,z=0))
    pose_stamped = pygazebo.msg.poses_stamped_pb2.PosesStamped(pose=[pose0,zeropose],time=ltime)
    #zeropose_stamped = pygazebo.msg.poses_stamped_pb2.PosesStamped(pose=[zeropose], time=ltime)
    for i in range(1):
        for j in range(20):
            yield From(posestamped_publisher.publish(pose_stamped))
        #yield From(posestamped_publisher.publish(zeropose_stamped))
        yield From(visual_publisher.publish(lvisuallink1))
        yield From(visual_publisher.publish(lvisuallink2))
        yield From(visual_publisher.publish(lvisualcollision1))
        yield From(visual_publisher.publish(lvisualmodel))
        #yield From(model_publisher.publish(pygazebo.msg.model_pb2.Model(name="unit_box1",id=150,is_static=False,pose=pose0,link=[llink],visual=[lvisualmodel],scale=vector111,self_collide=False))) 
        model=model_publisher.publish(pygazebo.msg.model_pb2.Model(name="unit_box1",id=150,is_static=False,pose=pose0,link=[llink],visual=[lvisualmodel],scale=vector111,self_collide=False))
        yield From(model_v_publisher.publish(pygazebo.msg.model_v_pb2.Model_V(models=[model])))


        print "add_box1"   
        yield From(trollius.sleep(1))
    
# logging.basicConfig(level=logging.ERROR)
loop = trollius.get_event_loop()
loop.run_until_complete(main())