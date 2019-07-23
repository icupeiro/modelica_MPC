within BuildingMpc.Examples.ControllerModels;
model Case900GEOTABS_cooling
  "Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
  extends Modelica.Icons.Example;
  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare package Medium = BuildingMpc.Media.DryAir,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                    airModel(massDynamics=
          Modelica.Fluid.Types.Dynamics.SteadyState),
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                            conTypFlo,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypA,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypB,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypC,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                           conTypD,
    hasWinA=true,
    fracA=0,
    redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
      shaTypA,
    hasWinB=false,
    hasWinC=false,
    hasWinD=false,
    bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    aziA=IDEAS.Types.Azimuth.S,
    mSenFac=0.822,
    l=8,
    w=6,
    A_winA=12,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
      glazingA,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
    redeclare IDEAS.Buildings.Components.InterzonalAirFlow.AirTight
      interzonalAirFlow,
    T_start=294.15)
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        BuildingMpc.Media.DryAir)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Blocks.Interfaces.RealInput   optVar1(
    start=0.5,
    min=0.05,
    max=0.5,
    nominal=0.5)
    annotation (Placement(transformation(extent={{-110,-20},{-90,0}})));
  IDEAS.Buildings.Components.BoundaryWall boundaryWall(
    inc=IDEAS.Types.Tilt.Floor,
    azi=rectangularZoneTemplate.aziA,
    A=rectangularZoneTemplate.A,
    redeclare Data.HeavyFloorTABS constructionType,
    port_emb(T(nominal=295)))    annotation (Placement(transformation(
        extent={{-6,-10},{6,10}},
        rotation=90,
        origin={-8,-38})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    A_floor=rectangularZoneTemplate.A,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    dp_nominal=10)
    annotation (Placement(transformation(extent={{6,-70},{26,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=2,
    p=200000,
    T=283.95) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_tabs(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04) "Pressure source"
    annotation (Placement(transformation(extent={{-4,-110},{-24,-90}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-4) "Pressure source"
    annotation (Placement(transformation(extent={{40,62},{60,82}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04) "Pressure source"
    annotation (Placement(transformation(extent={{156,-116},{136,-96}})));
public
  Modelica.Blocks.Interfaces.RealInput   optVar2(
    start=0.5,
    min=0.05,
    max=0.5,
    nominal=0.5)
    annotation (Placement(transformation(extent={{-110,34},{-90,54}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,-50})));
  Fluid.HeatPumps.HeatPump_y heatPump(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    dp2_nominal=0,
    dp1_nominal=0,
    m1_flow_nominal=0.5,
    m2_flow_nominal=0.5,
    Q_nom=3000,
    PLos=100,
    etaCom=1,
    COP_expr=5,
    Q_con_max=3000)
    annotation (Placement(transformation(extent={{86,-44},{106,-64}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(
    filDat=IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(),
    soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(),
    conDat=IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example(
        mBor_flow_nominal=0.5,
        dp_nominal=0,
        nBor=1,
        cooBor={{0,0}}))
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    V=1,
    nPorts=4,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    dynBal(hOut(nominal=146000)),
    T_start=308.15)
    annotation (Placement(transformation(extent={{54,-100},{74,-80}})));
  Modelica.Blocks.Sources.RealExpression TMeas(y=0.5)
    annotation (Placement(transformation(extent={{84,-98},{104,-78}})));
  Modelica.Blocks.Interfaces.RealInput   optVar3(
    min=0.05,
    max=1,
    start=0.1,
    nominal=1)
    annotation (Placement(transformation(extent={{-110,8},{-90,28}})));
  IDEAS.Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    m2_flow_nominal=0.5,
    dp1_nominal=0,
    dp2_nominal=0,
    eps=0.9,
    allowFlowReversal1=false,
    allowFlowReversal2=false,
    show_T=true,
    m1_flow_nominal=0.5)
                 annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,-24})));
  Modelica.Blocks.Sources.Constant       optVar4(k=1)
    annotation (Placement(transformation(extent={{-110,-94},{-90,-74}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_bor_in(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    tau=0,
    m_flow_nominal=0.5)
           annotation (Placement(transformation(
        extent={{6.5,-8.5},{-6.5,8.5}},
        rotation=-90,
        origin={65.5,19.5})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_PC_sup(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    tau=0,
    m_flow_nominal=0.5) annotation (Placement(transformation(
        extent={{-6.5,-8.5},{6.5,8.5}},
        rotation=0,
        origin={147.5,-38.5})));
  IDEAS.Fluid.FixedResistances.Junction jun(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    m_flow_nominal={1,1,1},
    dp_nominal={10,10,10})
    annotation (Placement(transformation(extent={{32,-68},{48,-52}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveMotor(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{30,-110},{10,-90}})));
  Modelica.Blocks.Interfaces.RealInput slack(min=0)
    annotation (Placement(transformation(extent={{-110,-142},{-90,-122}})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
        points={{16,-50},{16,-38},{2,-38}},          color={191,0,0}));
  connect(m_flow_source.port_b, heatPump.port_a2)
    annotation (Line(points={{60,72},{106,72},{106,-48}}, color={0,127,255}));
  connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,72},
          {-26,72},{-26,72},{40,72}}, color={0,127,255}));
  connect(boundaryWall.propsBus_a, rectangularZoneTemplate.proBusFlo[1])
    annotation (Line(
      points={{-10,-33},{-10,-16}},
      color={255,204,51},
      thickness=0.5));
  connect(heatPump.port_a1, vol.ports[1]) annotation (Line(points={{86,-60},{64,
          -60},{64,-100},{61,-100}}, color={0,127,255}));
  connect(m_flow_sink.port_b, vol.ports[2]) annotation (Line(points={{136,-106},
          {63,-106},{63,-100}}, color={0,127,255}));
  connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{156,-106},
          {156,-60},{106,-60}}, color={0,127,255}));
  connect(TMeas.y, m_flow_sink.m_flow_in) annotation (Line(points={{105,-88},{
          152,-88},{152,-98}},                  color={0,0,127}));
  connect(hex.port_b2, T_PC_sup.port_a) annotation (Line(points={{96,-34},{96,-38.5},
          {141,-38.5}},        color={0,127,255}));
  connect(jun.port_1, embeddedPipe.port_b)
    annotation (Line(points={{32,-60},{26,-60}}, color={0,127,255}));
  connect(jun.port_2, vol.ports[3])
    annotation (Line(points={{48,-60},{65,-60},{65,-100}}, color={0,127,255}));
  connect(jun.port_3, hex.port_a2) annotation (Line(points={{40,-68},{40,-136},{
          178,-136},{178,-14},{96,-14}},  color={0,127,255}));
  connect(sink.ports[1], embeddedPipe.port_a)
    annotation (Line(points={{-78,-60},{6,-60}}, color={0,127,255}));
  connect(threeWayValveMotor.port_b, m_flow_tabs.port_a)
    annotation (Line(points={{10,-100},{-4,-100}}, color={0,127,255}));
  connect(m_flow_tabs.port_b, embeddedPipe.port_a) annotation (Line(points={{
          -24,-100},{-32,-100},{-32,-60},{6,-60}}, color={0,127,255}));
  connect(threeWayValveMotor.port_a1, vol.ports[4])
    annotation (Line(points={{30,-100},{67,-100}}, color={0,127,255}));
  connect(threeWayValveMotor.port_a2, T_PC_sup.port_b) annotation (Line(points=
          {{20,-110},{20,-128},{168,-128},{168,-38.5},{154,-38.5}}, color={0,
          127,255}));
  connect(T_bor_in.port_b, source.ports[2]) annotation (Line(points={{65.5,26},
          {66,26},{66,42},{20,42},{20,68},{-26,68}}, color={0,127,255}));
  connect(m_flow_tabs.m_flow_in, optVar1) annotation (Line(points={{-8,-92},{-8,
          -28},{-52,-28},{-52,-10},{-100,-10}}, color={0,0,127}));
  connect(m_flow_source.m_flow_in, optVar2) annotation (Line(points={{44,80},{-12,
          80},{-12,86},{-68,86},{-68,44},{-100,44}}, color={0,0,127}));
  connect(optVar3, heatPump.y) annotation (Line(points={{-100,18},{-40,18},{-40,
          20},{86,20},{86,-63}}, color={0,0,127}));
  connect(optVar4.y, threeWayValveMotor.ctrl) annotation (Line(points={{-89,-84},
          {-34,-84},{-34,-89.2},{20,-89.2}}, color={0,0,127}));
  connect(hex.port_a1, heatPump.port_b2) annotation (Line(points={{84,-34},{84,
          -42},{84,-48},{86,-48}}, color={0,127,255}));
  connect(hex.port_b1, T_bor_in.port_a) annotation (Line(points={{84,-14},{76,
          -14},{76,-12},{65.5,-12},{65.5,13}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -140},{200,100}})),                                  Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-140},{200,
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
end Case900GEOTABS_cooling;
