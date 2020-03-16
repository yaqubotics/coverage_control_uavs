sdf_template = """<sdf version="1.4">
  <model name="_MODELNAME_">
    <static>0</static>
    <link name="cube_link">

      <inertial>
        <mass>_MASS_</mass>
        <inertia>
          <ixx>_IXX_</ixx>
          <ixy>0.000000</ixy>
          <ixz>0.000000</ixz>
          <iyy>_IYY_</iyy>
          <iyz>0.000000</iyz>
          <izz>_IZZ_</izz>
        </inertia>
      </inertial>

      <collision name="colision1">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>_SIZEX_ _SIZEY_ _SIZEZ_</size>
          </box>
        </geometry>
        <surface>
          <bounce>
            <restitution_coefficient>_RESTITUTION_COEFFICIENT_</restitution_coefficient>
            <threshold>_THRESHOLD_</threshold>
          </bounce>
          <friction>
            <ode>
              <mu>_MU_</mu>
              <mu2>_MU2_</mu2>
              <fdir1>_FDIR1X_ _FDIR1Y_ _FDIR1Z_</fdir1>
              <slip1>_SLIP1_</slip1>
              <slip2>_SLIP2_</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>_SOFT_CFM_</soft_cfm>
              <soft_erp>_SOFT_ERP_</soft_erp>
              <kp>_KP_</kp>
              <kd>_KD_</kd>
              <min_depth>_MINDEPTH_</min_depth>
              <max_vel>_MAXVEL_</max_vel>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual1">
        <pose>0 0 0 0 0 0</pose>
        <geometry>
          <box>
            <size>_SIZEX_ _SIZEY_ _SIZEZ_</size>
          </box>
        </geometry>
      </visual>

      <velocity_decay>
        <linear>_VELDECAYLINEAR_</linear>
        <angular>_VELDECAYANGULAR_</angular>
      </velocity_decay>
      <self_collide>0</self_collide>
      <kinematic>0</kinematic>
      <gravity>1</gravity>
    </link>
  </model>
</sdf>
"""
