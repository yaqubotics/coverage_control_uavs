sdf_ramp_template = """<sdf version="1.4">
  <model name="ramp">
    <static>1</static>
    <link name="ramp_link">

      <inertial>
        <mass>200.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0.000000</ixy>
          <ixz>0.000000</ixz>
          <iyy>0.1</iyy>
          <iyz>0.000000</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>

      <collision name="colision1">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>5.0 3.0 2.0</size>
          </box>
        </geometry>
        <surface>
          <bounce>
          </bounce>
          <friction>
            <ode>
              <mu>_MURAMP_</mu>
              <mu2>_MU2RAMP_</mu2>
              <fdir1>_FDIR1XRAMP_ _FDIR1YRAMP_ _FDIR1ZRAMP_</fdir1>
              <slip1>_SLIP1RAMP_</slip1>
              <slip2>_SLIP2RAMP_</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>_SOFT_CFMRAMP_</soft_cfm>
              <soft_erp>_SOFT_ERPRAMP_</soft_erp>
              <kp>_KPRAMP_</kp>
              <kd>_KDRAMP_</kd>
              <min_depth>_MINDEPTHRAMP_</min_depth>
              <max_vel>_MAXVELRAMP_</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual1">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>5.0 3.0 2.0</size>
          </box>
        </geometry>
      </visual>

      <velocity_decay>
        <linear>0.0</linear>
        <angular>0.0</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""
