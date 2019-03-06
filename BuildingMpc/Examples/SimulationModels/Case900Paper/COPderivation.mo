within BuildingMpc.Examples.SimulationModels.Case900Paper;
model COPderivation
  package Water = IDEAS.Media.Water;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=281.15, X_a=0.3);

  IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    dp1_nominal=10000,
    dp2_nominal=10000,
    redeclare package ref = IDEAS.Media.Refrigerants.R410A,
    enable_temperature_protection=false,
    redeclare package Medium2 = Glycol,
    m1_flow_nominal=0.05,
    m2_flow_nominal=0.05,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    datHeaPum=
        IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.Viessmann_BW301A45_58kW_5_50COP_R410A(),

    scaling_factor=0.02108,
    T1_start=298.15,
    T2_start=283.15)                              annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={0,2})));

  IDEAS.Fluid.Sources.Boundary_pT sou(
    nPorts=1,
    redeclare package Medium = Water,
    p=200000,
    use_T_in=true,
    T=273.15)
    annotation (Placement(transformation(extent={{60,40},{40,60}})));
  IDEAS.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Water,
    nPorts=1,
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
    m_flow_nominal=0.05,
    T_start=298.15)
    annotation (Placement(transformation(extent={{54,-14},{74,6}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sou(
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    redeclare package Medium = Water,
    m_flow_nominal=0.05,
    T_start=283.15)
    annotation (Placement(transformation(extent={{-34,36},{-54,56}})));
  IDEAS.Fluid.Sources.Boundary_pT sin1(redeclare package Medium = Water, nPorts=1)
    annotation (Placement(transformation(extent={{26,-78},{46,-58}})));
  IDEAS.Fluid.Sources.Boundary_pT sou1(nPorts=1, redeclare package Medium =
        Water)
    annotation (Placement(transformation(extent={{60,68},{40,88}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    redeclare package Medium = Glycol,
    m_flow_nominal=pump_sou.m_flow_nominal,
    tau=0,
    T_start=283.15)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-22,24})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = Water,
    m_flow_nominal=pump_sin.m_flow_nominal,
    tau=0,
    T_start=298.15)                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,-4})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable1(
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments,
    table=[0,298.15; 10000,298.15; 20000,303.15])
    annotation (Placement(transformation(extent={{-120,-40},{-100,-20}})));
  Modelica.Blocks.Sources.CombiTimeTable combiTimeTable2(
    extrapolation=Modelica.Blocks.Types.Extrapolation.HoldLastPoint,
    smoothness=Modelica.Blocks.Types.Smoothness.LinearSegments,
    table=[0,273.15; 10000,283.15; 20000,283.15; 30000,273.15])
    annotation (Placement(transformation(extent={{100,40},{80,60}})));
  Modelica.Blocks.Sources.Constant mFlow(k=1)
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  Modelica.Blocks.Sources.RealExpression COPThe(y=heaPum.QCon_flow/heaPum.com.PThe)
    annotation (Placement(transformation(extent={{-32,-84},{-12,-64}})));
equation
  connect(sin.ports[1], heaPum.port_a1) annotation (Line(points={{-60,-34},{-36,
          -34},{-36,-4},{-10,-4}}, color={0,127,255}));
  connect(sou.ports[1], heaPum.port_a2) annotation (Line(points={{40,50},{26,50},
          {26,8},{10,8}}, color={0,127,255}));
  connect(pump_sin.port_b, sin1.ports[1])
    annotation (Line(points={{74,-4},{74,-68},{46,-68}}, color={0,127,255}));
  connect(pump_sou.port_b, sou1.ports[1]) annotation (Line(points={{-54,46},{
          -72,46},{-72,80},{40,80},{40,78}},
                                         color={0,127,255}));
  connect(heaPum.port_b2, T_eva_out.port_a)
    annotation (Line(points={{-10,8},{-22,8},{-22,14}}, color={0,127,255}));
  connect(T_eva_out.port_b, pump_sou.port_a)
    annotation (Line(points={{-22,34},{-22,46},{-34,46}}, color={0,127,255}));
  connect(heaPum.port_b1, T_con_out.port_a)
    annotation (Line(points={{10,-4},{22,-4}}, color={0,127,255}));
  connect(T_con_out.port_b, pump_sin.port_a)
    annotation (Line(points={{42,-4},{54,-4}}, color={0,127,255}));
  connect(combiTimeTable1.y[1], sin.T_in) annotation (Line(points={{-99,-30},{
          -90,-30},{-90,-30},{-82,-30}}, color={0,0,127}));
  connect(combiTimeTable2.y[1], sou.T_in) annotation (Line(points={{79,50},{72,
          50},{72,54},{62,54}}, color={0,0,127}));
  connect(mFlow.y, heaPum.y) annotation (Line(points={{-79,50},{-76,50},{-76,-1},
          {-12,-1}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end COPderivation;
