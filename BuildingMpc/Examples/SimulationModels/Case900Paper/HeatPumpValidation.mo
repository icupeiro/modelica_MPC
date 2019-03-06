within BuildingMpc.Examples.SimulationModels.Case900Paper;
model HeatPumpValidation
  package Water = IDEAS.Media.Water;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=281.15, X_a=0.3);

  IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    dp1_nominal=10000,
    dp2_nominal=10000,
    redeclare package ref = IDEAS.Media.Refrigerants.R410A,
    datHeaPum=
        IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.Daikin_WRA036_13kW_4_50COP_R410A(),
    enable_temperature_protection=false,
    redeclare package Medium2 = Glycol,
    scaling_factor=0.12684,
    m1_flow_nominal=0.05,
    m2_flow_nominal=0.05)                         annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={0,2})));

  IDEAS.Fluid.Sources.Boundary_pT sou(
    nPorts=2,
    redeclare package Medium = Water,
    p=200000,
    T=278.15)
    annotation (Placement(transformation(extent={{60,40},{40,60}})));
  IDEAS.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Water,
    nPorts=2,
    use_T_in=true,
    p=200000)
    annotation (Placement(transformation(extent={{-80,-44},{-60,-24}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sin(
    redeclare package Medium = Water,
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{54,-14},{74,6}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sou(
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    redeclare package Medium = Water,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{-34,36},{-54,56}})));
  IDEAS.Fluid.Sources.Boundary_pT sin1(redeclare package Medium = Water, nPorts=2)
    annotation (Placement(transformation(extent={{26,-78},{46,-58}})));
  IDEAS.Fluid.Sources.Boundary_pT sou1(nPorts=2, redeclare package Medium =
        Water)
    annotation (Placement(transformation(extent={{60,68},{40,88}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    redeclare package Medium = Glycol,
    m_flow_nominal=pump_sou.m_flow_nominal,
    tau=60) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-22,24})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = Water,
    m_flow_nominal=pump_sin.m_flow_nominal,
    tau=60)                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,-4})));
  Fluid.HeatPumps.HeatPump heatPump(
    m1_flow_nominal=pump_sin.m_flow_nominal,
    dp1_nominal=0,
    m2_flow_nominal=pump_sou.m_flow_nominal,
    dp2_nominal=0,
    redeclare package Medium1 = Water,
    redeclare package Medium2 = Glycol,
    Q_nom=885.86,
    PLoss=heaPum.com.PLos,
    etaCom=heaPum.com.etaEle)
    annotation (Placement(transformation(extent={{-8,-32},{12,-52}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sin1(
    redeclare package Medium = Water,
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{78,-52},{98,-32}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out1(
    redeclare package Medium = Water,
    m_flow_nominal=pump_sin.m_flow_nominal,
    tau=60)                                 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={56,-42})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out1(
    redeclare package Medium = Glycol,
    m_flow_nominal=pump_sou.m_flow_nominal,
    tau=60) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-28,-26})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sou1(
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    redeclare package Medium = Water,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{-46,-12},{-66,8}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable(
                       smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    table=[0,313.15; 7200,318.15; 10800,323.15])
    annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
  IDEAS.Controls.Continuous.LimPID conPID(
    Td=0.5,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.5,
    Ti=10)  annotation (Placement(transformation(extent={{-68,16},{-54,30}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable1(
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    table=[0,298.15; 7200,303.15; 10800,308.15; 16000,313.15])
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));
equation
  connect(sin.ports[1], heaPum.port_a1) annotation (Line(points={{-60,-32},{-36,
          -32},{-36,-4},{-10,-4}}, color={0,127,255}));
  connect(sou.ports[1], heaPum.port_a2) annotation (Line(points={{40,52},{26,52},
          {26,8},{10,8}}, color={0,127,255}));
  connect(pump_sin.port_b, sin1.ports[1])
    annotation (Line(points={{74,-4},{74,-66},{46,-66}}, color={0,127,255}));
  connect(pump_sou.port_b, sou1.ports[1]) annotation (Line(points={{-54,46},{-72,
          46},{-72,80},{40,80},{40,80}}, color={0,127,255}));
  connect(heaPum.port_b2, T_eva_out.port_a)
    annotation (Line(points={{-10,8},{-22,8},{-22,14}}, color={0,127,255}));
  connect(T_eva_out.port_b, pump_sou.port_a)
    annotation (Line(points={{-22,34},{-22,46},{-34,46}}, color={0,127,255}));
  connect(heaPum.port_b1, T_con_out.port_a)
    annotation (Line(points={{10,-4},{22,-4}}, color={0,127,255}));
  connect(T_con_out.port_b, pump_sin.port_a)
    annotation (Line(points={{42,-4},{54,-4}}, color={0,127,255}));
  connect(heatPump.port_b1, T_con_out1.port_a) annotation (Line(points={{12,-48},
          {30,-48},{30,-42},{46,-42}}, color={0,127,255}));
  connect(T_con_out1.port_b, pump_sin1.port_a)
    annotation (Line(points={{66,-42},{78,-42}}, color={0,127,255}));
  connect(pump_sin1.port_b, sin1.ports[2])
    annotation (Line(points={{98,-42},{98,-70},{46,-70}}, color={0,127,255}));
  connect(heatPump.port_b2, T_eva_out1.port_a)
    annotation (Line(points={{-8,-36},{-28,-36}}, color={0,127,255}));
  connect(T_eva_out1.port_b, pump_sou1.port_a)
    annotation (Line(points={{-28,-16},{-28,-2},{-46,-2}}, color={0,127,255}));
  connect(pump_sou1.port_b, sou1.ports[2]) annotation (Line(points={{-66,-2},{-94,
          -2},{-94,76},{40,76}}, color={0,127,255}));
  connect(sou.ports[2], heatPump.port_a2) annotation (Line(points={{40,48},{42,48},
          {42,-36},{12,-36}}, color={0,127,255}));
  connect(T_con_out.T, conPID.u_m)
    annotation (Line(points={{32,7},{32,14.6},{-61,14.6}}, color={0,0,127}));
  connect(conPID.y, heaPum.y) annotation (Line(points={{-53.3,23},{-33.65,23},{-33.65,
          -1},{-12,-1}}, color={0,0,127}));
  connect(combiTimeTable.y[1], heatPump.Tcon_out) annotation (Line(points={{-59,
          -70},{-34,-70},{-34,-51},{-8,-51}}, color={0,0,127}));
  connect(combiTimeTable.y[1], conPID.u_s) annotation (Line(points={{-59,-70},{-86,
          -70},{-86,23},{-69.4,23}}, color={0,0,127}));
  connect(sin.ports[2], heatPump.port_a1) annotation (Line(points={{-60,-36},{-34,
          -36},{-34,-48},{-8,-48}}, color={0,127,255}));
  connect(combiTimeTable1.y[1], sin.T_in) annotation (Line(points={{-99,-30},{-90,
          -30},{-90,-30},{-82,-30}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatPumpValidation;
