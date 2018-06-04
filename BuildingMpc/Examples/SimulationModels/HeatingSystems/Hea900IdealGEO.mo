within BuildingMpc.Examples.SimulationModels.HeatingSystems;
model Hea900IdealGEO "ideal geothermal system for case 900"
  extends IDEAS.Templates.Interfaces.BaseClasses.HeatingSystem(
  nZones=1,
  nLoads=1,
  nEmbPorts=1,
  nTemSen=1,
  isCoo=true,
  nConvPorts=1,
  nRadPorts=1,
  P={0},
  Q={0},
  Q_design={2570},
  QHeaSys=max(0,heatPortEmb[1].Q_flow),
  QCooTotal=min(0,heatPortEmb[1].Q_flow));
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
    nPorts=4,
    T_start=308.15)
              annotation (Placement(transformation(extent={{-78,38},{-58,18}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor watCon(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    m_flow_nominal=sink_pump.m_flow_nominal)
    "3way valve to control water temperature"
    annotation (Placement(transformation(extent={{-78,70},{-98,90}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor pasCooCon(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    m_flow_nominal=sink_pump.m_flow_nominal)
    "3way valve to control passive cooling"
    annotation (Placement(transformation(extent={{-104,70},{-124,90}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow tabs_pump(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    tau=60,
    use_inputFilter=false,
    addPowerToMedium=false,
    m_flow_nominal=0.125,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{-130,90},{-150,70}})));
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
    annotation (Placement(transformation(extent={{112,96},{96,112}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=T_con_out.T -
        T_con_in.T)
    annotation (Placement(transformation(extent={{150,82},{130,102}})));
  Modelica.Blocks.Sources.Constant sink_set(k=5)
    annotation (Placement(transformation(extent={{144,122},{130,136}})));
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
  Modelica.Blocks.Sources.RealExpression heatingMode(y=if runMean.TRm < (273.15
         + 12.0) then 0.0 else 1.0)
    annotation (Placement(transformation(extent={{-140,104},{-120,124}})));
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
  IDEAS.Controls.ControlHeating.RunningMeanTemperatureEN15251 runMean(each
      TAveDayIni=10.0*ones(7))
    annotation (Placement(transformation(extent={{-178,-92},{-158,-72}})));
  IDEAS.Controls.Continuous.LimPID heaCurve(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=1,
    Ti=60,
    yMax=1,
    yMin=0,
    reverseAction=false)
    annotation (Placement(transformation(extent={{-68,108},{-84,124}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=T_tabs_in.T)
    annotation (Placement(transformation(extent={{-38,90},{-58,110}})));
  Modelica.Blocks.Tables.CombiTable1D heaCombi(table=[273.15 - 8,273.15 + 35;
        273.15 + 22,273.15 + 22])
    annotation (Placement(transformation(extent={{-18,126},{-38,146}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=sim.Te)
    annotation (Placement(transformation(extent={{24,126},{4,146}})));
  Modelica.Blocks.Sources.RealExpression tempControl(y=if runMean.TRm < (273.15
         + 12.0) then heaCurve.y else 1.0)
    annotation (Placement(transformation(extent={{-140,130},{-120,150}})));
  Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=35, uHigh=40)
    annotation (Placement(transformation(extent={{-50,-38},{-30,-18}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{-80,-38},{-60,-18}})));
  Modelica.Blocks.Logical.Not not1
    annotation (Placement(transformation(extent={{-22,-36},{-6,-20}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=if hysteresis.y
         then PID_HP_sink.y else 0)
    annotation (Placement(transformation(extent={{58,92},{38,112}})));
  Modelica.Blocks.Sources.Constant sink_set1(k=273.15 + 40)
    annotation (Placement(transformation(extent={{74,122},{60,136}})));
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
  connect(embeddedPipe.heatPortEmb[1], heatPortEmb[1]) annotation (Line(points={{-188,60},
          {-194,60},{-200,60}},                     color={191,0,0}));
  connect(T_con_out.port_b, buffTank.ports[2]) annotation (Line(points={{-28,80},
          {-68,80},{-68,38},{-69,38}}, color={0,127,255}));
  connect(watCon.port_b, pasCooCon.port_a1) annotation (Line(points={{-98,80},{-101,
          80},{-104,80}},      color={0,127,255}));
  connect(pasCooCon.port_a2, passiveCooling.port_b1) annotation (Line(points={{-114,70},
          {-114,70},{-114,68},{-114,8},{-38,8}},     color={0,127,255}));
  connect(sink.ports[1], T_con_out.port_a)
    annotation (Line(points={{94,74},{94,80},{-8,80}}, color={0,127,255}));
  connect(source_set.y, PID_HP_source.u_s) annotation (Line(points={{151.4,66},{
          144,66},{144,40},{135.6,40}}, color={0,0,127}));
  connect(realExpression.y, PID_HP_source.u_m)
    annotation (Line(points={{151,28},{126,28},{126,30.4}}, color={0,0,127}));
  connect(PID_HP_source.y, source_pump.m_flow_in)
    annotation (Line(points={{117.2,40},{102,40},{102,28}}, color={0,0,127}));
  connect(realExpression1.y, PID_HP_sink.u_m)
    annotation (Line(points={{129,92},{104,92},{104,94.4}},
                                                         color={0,0,127}));
  connect(sink_set.y, PID_HP_sink.u_s) annotation (Line(points={{129.3,129},{
          124,129},{124,130},{120,130},{120,104},{113.6,104}},
                              color={0,0,127}));
  connect(T_tabs_in.port_b, embeddedPipe.port_a) annotation (Line(points={{-176,
          80},{-178,80},{-178,70}}, color={0,127,255}));
  connect(embeddedPipe.port_b, T_tabs_out.port_a)
    annotation (Line(points={{-178,50},{-178,48}}, color={0,127,255}));
  connect(T_tabs_out.port_b, buffTank.ports[3]) annotation (Line(points={{-178,28},
          {-118,28},{-108,28},{-88,28},{-88,38},{-67,38}}, color={0,127,255}));
  connect(passiveCooling.port_a1, T_tabs_out.port_b) annotation (Line(points={{-38,
          -12},{-100,-12},{-178,-12},{-178,28}}, color={0,127,255}));
  connect(watCon.port_a2, T_tabs_out.port_b) annotation (Line(points={{-88,70},
          {-88,70},{-88,28},{-178,28}},   color={0,127,255}));
  connect(heatingMode.y, pasCooCon.ctrl) annotation (Line(points={{-119,114},{
          -114,114},{-114,90.8}},
                            color={0,0,127}));
  connect(realExpression2.y, PID_HP_sink1.u_m) annotation (Line(points={{-33,-74},
          {-58,-74},{-58,-71.6}}, color={0,0,127}));
  connect(tabs_set.y, PID_HP_sink1.u_s) annotation (Line(points={{13,-44},{-4,-44},
          {-4,-62},{-48.4,-62}}, color={0,0,127}));
  connect(PID_HP_sink1.y, tabs_pump.m_flow_in) annotation (Line(points={{-66.8,-62},
          {-140,-62},{-140,68}},
                               color={0,0,127}));
  connect(watCon.port_a1, buffTank.ports[4]) annotation (Line(points={{-78,80},
          {-76,80},{-72,80},{-72,60},{-72,38},{-70,38},{-66,38},{-65,38}},
                              color={0,127,255}));
  connect(pasCooCon.port_b, tabs_pump.port_a)
    annotation (Line(points={{-124,80},{-130,80}}, color={0,127,255}));
  connect(T_tabs_in.port_a, tabs_pump.port_b)
    annotation (Line(points={{-156,80},{-150,80}}, color={0,127,255}));
   heatPortCon.Q_flow = {0};
   heatPortRad.Q_flow = {0};
  connect(realExpression3.y, heaCurve.u_m) annotation (Line(points={{-59,100},{
          -76,100},{-76,106.4}}, color={0,0,127}));
  connect(heaCombi.y[1], heaCurve.u_s) annotation (Line(points={{-39,136},{-52,
          136},{-52,116},{-66.4,116}}, color={0,0,127}));
  connect(realExpression4.y, heaCombi.u[1])
    annotation (Line(points={{3,136},{-16,136}}, color={0,0,127}));
  connect(tempControl.y, watCon.ctrl) annotation (Line(points={{-119,140},{-104,
          140},{-104,90.8},{-88,90.8}}, color={0,0,127}));
  connect(buffTank.heatPort, temperatureSensor.port)
    annotation (Line(points={{-78,28},{-80,28},{-80,-28}}, color={191,0,0}));
  connect(temperatureSensor.T, hysteresis.u)
    annotation (Line(points={{-60,-28},{-52,-28}}, color={0,0,127}));
  connect(hysteresis.y, not1.u)
    annotation (Line(points={{-29,-28},{-23.6,-28}}, color={255,0,255}));
  connect(realExpression5.y, sink_pump.m_flow_in) annotation (Line(points={{37,
          102},{14,102},{14,50},{-40,50}}, color={0,0,127}));
  connect(sink_set1.y, heaPum.TSet)
    annotation (Line(points={{59.3,129},{10,129},{10,44.1}}, color={0,0,127}));
end Hea900IdealGEO;
