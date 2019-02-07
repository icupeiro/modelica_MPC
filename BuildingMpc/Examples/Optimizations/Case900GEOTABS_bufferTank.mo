within BuildingMpc.Examples.Optimizations;
model Case900GEOTABS_bufferTank
  "Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
  extends Modelica.Icons.Example;
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  Modelica.Blocks.Sources.RealExpression optVar1(y=mpcCase900GEOTABS_bufferTank.yOpt[
        1]) annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    dp_nominal=0,
    m_flow_nominal=0.5,
    A_floor=case900FloorHeating.floor.A)
    annotation (Placement(transformation(extent={{-6,-70},{14,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    p=200000,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
  Modelica.Blocks.Sources.RealExpression optVar3(y=mpcCase900GEOTABS_bufferTank.yOpt[
        3]) annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
protected
  IDEAS.Fluid.Movers.FlowControlled_m_flow   m_flow_tabs(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final m_flow_small=1e-04,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    nominalValuesDefineDefaultPressureCurve=true) "Pressure source"
    annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
protected
  IDEAS.Fluid.Movers.FlowControlled_m_flow   m_flow_source(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final m_flow_small=1e-4,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=1.2,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    nominalValuesDefineDefaultPressureCurve=true)
                             "Pressure source"
    annotation (Placement(transformation(extent={{40,62},{60,82}})));
protected
  IDEAS.Fluid.Movers.FlowControlled_m_flow m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final m_flow_small=1e-04,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    nominalValuesDefineDefaultPressureCurve=true,
    inputType=IDEAS.Fluid.Types.InputType.Stages) "Pressure source"
    annotation (Placement(transformation(extent={{128,-136},{108,-116}})));
public
  Modelica.Blocks.Sources.RealExpression optVar2(y=mpcCase900GEOTABS_bufferTank.yOpt[
        2]) annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,-50})));
  IBPSA.Fluid.HeatPumps.ScrollWaterToWater
                                       heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    dp2_nominal=0,
    dp1_nominal=0,
    m1_flow_nominal=0.5,
    m2_flow_nominal=1.2,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    enable_variable_speed=false,
    scaling_factor=3000/10200,
    datHeaPum=
        IBPSA.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A())
    annotation (Placement(transformation(extent={{60,-44},{80,-64}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(
    filDat=IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(),
    soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(),
    conDat=IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example())
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  IBPSA.Fluid.Geothermal.Borefields.OneUTube       multipleBorehole(
    redeclare package Medium = IDEAS.Media.Water,
    borFieDat=borFieDat,
    TExt0_start=(273.15 + 13.5),
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dynFil=true,
    r={3},
    nbTem=1)
    annotation (Placement(transformation(extent={{52,0},{32,20}})));
  IDEAS.Templates.Structure.Case900FloorHeating case900FloorHeating
    annotation (Placement(transformation(extent={{-16,-20},{14,0}})));
  MPCs.MpcCase900GEOTABS_bufferTank
                         mpcCase900GEOTABS_bufferTank
    annotation (Placement(transformation(extent={{-94,20},{-74,40}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    allowFlowReversal=false,
    m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{158,-134},{140,-118}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = IDEAS.Media.Water,
    m_flow_nominal=0.5,
    nPorts=4,
    T_start=308.15,
    V=1)
    annotation (Placement(transformation(extent={{52,-92},{72,-72}})));
  Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=-3, uHigh=1)
    annotation (Placement(transformation(extent={{-16,4},{4,24}})));
  Modelica.Blocks.Math.Add add(k2=-1)
    annotation (Placement(transformation(extent={{-46,4},{-26,24}})));
  Modelica.Blocks.Math.BooleanToInteger booleanToInteger
    annotation (Placement(transformation(extent={{90,-112},{110,-92}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{16,-94},{-4,-74}})));
equation
  connect(m_flow_tabs.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-24,-60},{-6,-60}}, color={0,127,255}));
  connect(optVar1.y,m_flow_tabs. m_flow_in) annotation (Line(points={{-65,-10},
          {-40,-10},{-40,-48},{-34,-48}}, color={0,0,127}));
  connect(optVar2.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
          {-22,90},{50,90},{50,84}},        color={0,0,127}));
  connect(sink.ports[1],m_flow_tabs. port_a) annotation (Line(points={{-78,-60},
          {-78,-60},{-44,-60}}, color={0,127,255}));
  connect(m_flow_source.port_b, heaPum.port_a2) annotation (Line(points={{60,72},
          {60,72},{88,72},{88,-48},{80,-48}}, color={0,127,255}));
  connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
          {-26,72},{-26,72},{40,72}}, color={0,127,255}));
  connect(multipleBorehole.port_b, m_flow_source.port_a) annotation (Line(
        points={{32,10},{20,10},{20,72},{40,72}}, color={0,127,255}));
  connect(embeddedPipe.heatPortEmb[1], case900FloorHeating.heatPortEmb[1])
    annotation (Line(points={{4,-50},{4,-28},{14,-28},{14,-4}},
                                                             color={191,0,0}));

  connect(optVar3.y, add.u1) annotation (Line(points={{-65,-24},{-58,-24},{-58,
          20},{-48,20}}, color={0,0,127}));
  connect(add.y, hysteresis.u)
    annotation (Line(points={{-25,14},{-18,14}}, color={0,0,127}));
  connect(booleanToInteger.y, heaPum.stage) annotation (Line(points={{111,-102},
          {111,-43},{58,-43},{58,-57}},
                                      color={255,127,0}));
  connect(booleanToInteger.u, hysteresis.y)
    annotation (Line(points={{88,-102},{42,-102},{42,-12},{24,-12},{24,14},{5,
          14}},                                        color={255,0,255}));
  connect(booleanToInteger.y, m_flow_sink.stage) annotation (Line(points={{111,
          -102},{118,-102},{118,-114}},               color={255,127,0}));
  connect(m_flow_sink.port_b, vol.ports[1]) annotation (Line(points={{108,-126},
          {52,-126},{52,-92},{59,-92}}, color={0,127,255}));
  connect(heaPum.port_a1, vol.ports[2]) annotation (Line(points={{60,-60},{52,
          -60},{52,-92},{61,-92}}, color={0,127,255}));
  connect(m_flow_sink.port_a, T_con_out.port_b)
    annotation (Line(points={{128,-126},{140,-126}},
                                                   color={0,127,255}));
  connect(T_con_out.port_a, heaPum.port_b1) annotation (Line(points={{158,-126},
          {158,-60},{80,-60}},            color={0,127,255}));
  connect(vol.heatPort, temperatureSensor.port) annotation (Line(points={{52,
          -82},{35,-82},{35,-84},{16,-84}}, color={191,0,0}));
  connect(temperatureSensor.T, add.u2) annotation (Line(points={{-4,-84},{-14,
          -84},{-14,-32},{-30,-32},{-30,-4},{-48,-4},{-48,8}}, color={0,0,127}));
  connect(heaPum.port_b2, multipleBorehole.port_a) annotation (Line(points={{60,
          -48},{62,-48},{62,-2},{52,-2},{52,10}}, color={0,127,255}));
  connect(embeddedPipe.port_b, vol.ports[3]) annotation (Line(points={{14,-60},
          {32,-60},{32,-92},{63,-92}}, color={0,127,255}));
  connect(m_flow_tabs.port_a, vol.ports[4]) annotation (Line(points={{-44,-60},
          {-44,-112},{65,-112},{65,-92}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -180},{200,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-180},{200,
            100}})),
    experiment(
      StopTime=31536000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>", revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),
    __Dymola_Commands(file=
          "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
        "Simulate and plot"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(
        GenerateVariableDependencies=false,
        OutputModelicaCode=false,
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false));
end Case900GEOTABS_bufferTank;
