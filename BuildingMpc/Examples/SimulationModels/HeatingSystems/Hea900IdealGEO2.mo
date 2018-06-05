within BuildingMpc.Examples.SimulationModels.HeatingSystems;
model Hea900IdealGEO2 "ideal geothermal system for case 900"

  IDEAS.Fluid.HeatPumps.Carnot_TCon heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    QCon_flow_nominal=2570,
    dTEva_nominal=-3,
    dTCon_nominal=5,
    use_eta_Carnot_nominal=false,
    COP_nominal=4.5,
    dp1_nominal=0,
    dp2_nominal=0,
    TAppCon_nominal=5,
    TAppEva_nominal=3,
    QCon_flow_max=2570,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    m2_flow_nominal=source_pump.m_flow_nominal,
    TCon_nominal=308.15,
    TEva_nominal=276.15)
    annotation (Placement(transformation(extent={{-20,-19},{20,19}},
        rotation=0,
        origin={34,27})));
  IDEAS.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    nPorts=2,
    p=200000,
    T=289.15) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={132,-28})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow source_pump(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.16,
    tau=60,
    use_inputFilter=false,
    addPowerToMedium=false,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{112,6},{92,26}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_in(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=source_pump.m_flow_nominal)
    annotation (Placement(transformation(extent={{80,8},{64,24}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=source_pump.m_flow_nominal)
    annotation (Placement(transformation(extent={{4,8},{-14,24}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
    redeclare Data.TradDesignTABS RadSlaCha,
    m_flow_nominal=0.125,
    A_floor=48)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={-178,60})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow sink_pump(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    use_inputFilter=false,
    addPowerToMedium=false,
    m_flow_nominal=0.125,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{-50,28},{-30,48}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=sink_pump.m_flow_nominal)
    annotation (Placement(transformation(extent={{-20,30},{-4,46}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=sink_pump.m_flow_nominal)
    annotation (Placement(transformation(extent={{-8,72},{-28,88}})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness passiveCooling(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    m2_flow_nominal=source_pump.m_flow_nominal,
    dp1_nominal=0,
    dp2_nominal=0,
    eps=0.9,
    m1_flow_nominal=tabs_pump.m_flow_nominal) annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-32,-2})));
  IDEAS.Fluid.MixingVolumes.MixingVolume buffTank(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=sink_pump.m_flow_nominal,
    V=0.1,
    nPorts=4) annotation (Placement(transformation(extent={{-78,38},{-58,18}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor watCon(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    m_flow_nominal=sink_pump.m_flow_nominal)
    "3way valve to control water temperature"
    annotation (Placement(transformation(extent={{-108,70},{-128,90}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor pasCooCon(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    m_flow_nominal=sink_pump.m_flow_nominal)
    "3way valve to control passive cooling"
    annotation (Placement(transformation(extent={{-134,70},{-154,90}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow tabs_pump(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    use_inputFilter=false,
    addPowerToMedium=false,
    m_flow_nominal=0.125,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{-80,90},{-100,70}})));
  IDEAS.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={94,64})));
  IDEAS.Controls.Continuous.LimPID PID_HP_source(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1,
    Ti=60,
    yMax=source_pump.m_flow_nominal,
    yMin=source_pump.m_flow_nominal*0.1,
    reverseAction=true)
    annotation (Placement(transformation(extent={{134,32},{118,48}})));
  Modelica.Blocks.Sources.Constant source_set(k=3)
    annotation (Placement(transformation(extent={{164,60},{152,72}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=T_eva_in.T -
        T_eva_out.T)
    annotation (Placement(transformation(extent={{172,18},{152,38}})));
  IDEAS.Controls.Continuous.LimPID PID_HP_sink(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1,
    Ti=60,
    yMax=sink_pump.m_flow_nominal,
    yMin=sink_pump.m_flow_nominal*0.1)
    annotation (Placement(transformation(extent={{22,60},{6,76}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=T_con_out.T -
        T_con_in.T)
    annotation (Placement(transformation(extent={{60,46},{40,66}})));
  Modelica.Blocks.Sources.Constant sink_set(k=5)
    annotation (Placement(transformation(extent={{54,86},{40,100}})));
  Modelica.Blocks.Sources.Constant tabs_set(k=5)
    annotation (Placement(transformation(extent={{34,-54},{14,-34}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_tabs_in(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=tabs_pump.m_flow_nominal)
    annotation (Placement(transformation(extent={{-156,72},{-176,88}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_tabs_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=tabs_pump.m_flow_nominal) annotation (Placement(
        transformation(
        extent={{10,-8},{-10,8}},
        rotation=90,
        origin={-178,38})));
  Modelica.Blocks.Sources.RealExpression heatingMode(y=if sim.Te < 10 then 1
         else 0)
    annotation (Placement(transformation(extent={{-122,86},{-142,106}})));
  IDEAS.Controls.Continuous.LimPID PID_HP_sink1(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1,
    Ti=60,
    yMax=tabs_pump.m_flow_nominal,
    yMin=tabs_pump.m_flow_nominal*0.1)
    annotation (Placement(transformation(extent={{-50,-70},{-66,-54}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=T_tabs_in.T -
        T_tabs_out.T)
    annotation (Placement(transformation(extent={{-12,-84},{-32,-64}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=if heatingMode.y == 1.0
         then 1 else 0.5)
    annotation (Placement(transformation(extent={{-52,86},{-72,106}})));
  Modelica.Blocks.Sources.Constant tabs_set1(k=273.15 + 35)
    annotation (Placement(transformation(extent={{70,-56},{50,-36}})));
  inner IDEAS.BoundaryConditions.SimInfoManager sim
    annotation (Placement(transformation(extent={{-98,-98},{-78,-78}})));
equation
  connect(source_pump.port_a, source.ports[1])
    annotation (Line(points={{112,16},{130,16},{130,-18}}, color={0,127,255}));
  connect(heaPum.port_a2, T_eva_in.port_b)
    annotation (Line(points={{54,15.6},{54,16},{64,16}}, color={0,127,255}));
  connect(T_eva_in.port_a, source_pump.port_b)
    annotation (Line(points={{80,16},{80,16},{92,16}}, color={0,127,255}));
  connect(T_eva_out.port_a, heaPum.port_b2)
    annotation (Line(points={{4,16},{14,16},{14,15.6}}, color={0,127,255}));
  connect(T_con_in.port_b, heaPum.port_a1)
    annotation (Line(points={{-4,38},{14,38},{14,38.4}}, color={0,127,255}));
  connect(heaPum.port_b1, T_con_out.port_a) annotation (Line(points={{54,38.4},{
          64,38.4},{64,80},{-8,80}}, color={0,127,255}));
  connect(sink_pump.port_b, T_con_in.port_a)
    annotation (Line(points={{-30,38},{-20,38}}, color={0,127,255}));
  connect(passiveCooling.port_a2, T_eva_out.port_b)
    annotation (Line(points={{-26,8},{-26,16},{-14,16}}, color={0,127,255}));
  connect(passiveCooling.port_b2, source.ports[2]) annotation (Line(points={{-26,
          -12},{-26,-18},{134,-18}}, color={0,127,255}));
  connect(buffTank.ports[1], sink_pump.port_a)
    annotation (Line(points={{-71,38},{-71,38},{-50,38}}, color={0,127,255}));
  connect(T_con_out.port_b, buffTank.ports[2]) annotation (Line(points={{-28,80},
          {-68,80},{-68,38},{-69,38}}, color={0,127,255}));
  connect(buffTank.ports[3], tabs_pump.port_a) annotation (Line(points={{-67,38},
          {-70,38},{-70,80},{-80,80}}, color={0,127,255}));
  connect(tabs_pump.port_b, watCon.port_a1) annotation (Line(points={{-100,80},{
          -104,80},{-108,80}}, color={0,127,255}));
  connect(watCon.port_b, pasCooCon.port_a1) annotation (Line(points={{-128,80},{
          -131,80},{-134,80}}, color={0,127,255}));
  connect(pasCooCon.port_a2, passiveCooling.port_b1) annotation (Line(points={{-144,
          70},{-144,70},{-144,12},{-144,8},{-38,8}}, color={0,127,255}));
  connect(sink.ports[1], T_con_out.port_a)
    annotation (Line(points={{94,74},{94,80},{-8,80}}, color={0,127,255}));
  connect(source_set.y, PID_HP_source.u_s) annotation (Line(points={{151.4,66},{
          144,66},{144,40},{135.6,40}}, color={0,0,127}));
  connect(realExpression.y, PID_HP_source.u_m)
    annotation (Line(points={{151,28},{126,28},{126,30.4}}, color={0,0,127}));
  connect(PID_HP_source.y, source_pump.m_flow_in)
    annotation (Line(points={{117.2,40},{102,40},{102,28}}, color={0,0,127}));
  connect(realExpression1.y, PID_HP_sink.u_m)
    annotation (Line(points={{39,56},{14,56},{14,58.4}}, color={0,0,127}));
  connect(sink_set.y, PID_HP_sink.u_s) annotation (Line(points={{39.3,93},{32,93},
          {32,68},{23.6,68}}, color={0,0,127}));
  connect(PID_HP_sink.y, sink_pump.m_flow_in) annotation (Line(points={{5.2,68},
          {0,68},{0,60},{-40,60},{-40,50}}, color={0,0,127}));
  connect(pasCooCon.port_b, T_tabs_in.port_a)
    annotation (Line(points={{-154,80},{-156,80}}, color={0,127,255}));
  connect(T_tabs_in.port_b, embeddedPipe.port_a) annotation (Line(points={{-176,
          80},{-178,80},{-178,70}}, color={0,127,255}));
  connect(embeddedPipe.port_b, T_tabs_out.port_a)
    annotation (Line(points={{-178,50},{-178,48}}, color={0,127,255}));
  connect(T_tabs_out.port_b, buffTank.ports[4]) annotation (Line(points={{-178,28},
          {-118,28},{-118,28},{-92,28},{-92,38},{-65,38}}, color={0,127,255}));
  connect(passiveCooling.port_a1, T_tabs_out.port_b) annotation (Line(points={{-38,
          -12},{-100,-12},{-178,-12},{-178,28}}, color={0,127,255}));
  connect(watCon.port_a2, T_tabs_out.port_b) annotation (Line(points={{-118,70},
          {-118,70},{-118,28},{-178,28}}, color={0,127,255}));
  connect(heatingMode.y, pasCooCon.ctrl) annotation (Line(points={{-143,96},{-144,
          96},{-144,90.8}}, color={0,0,127}));
  connect(realExpression2.y, PID_HP_sink1.u_m) annotation (Line(points={{-33,-74},
          {-58,-74},{-58,-71.6}}, color={0,0,127}));
  connect(tabs_set.y, PID_HP_sink1.u_s) annotation (Line(points={{13,-44},{-4,-44},
          {-4,-62},{-48.4,-62}}, color={0,0,127}));
  connect(PID_HP_sink1.y, tabs_pump.m_flow_in) annotation (Line(points={{-66.8,-62},
          {-90,-62},{-90,68}}, color={0,0,127}));
  connect(realExpression3.y, watCon.ctrl) annotation (Line(points={{-73,96},{-94,
          96},{-118,96},{-118,90.8}},          color={0,0,127}));
  connect(tabs_set1.y, heaPum.TSet) annotation (Line(points={{49,-46},{46,-46},{
          46,-6},{10,-6},{10,44.1}}, color={0,0,127}));
end Hea900IdealGEO2;
