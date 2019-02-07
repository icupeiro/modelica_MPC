within BuildingMpc.Examples.ControllerModels;
model Case900GEOTABS_bufferTank
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
    T_start=293.15)
    annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
  inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
    "Simulation information manager for climate data"
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
        BuildingMpc.Media.DryAir)
    annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
  Modelica.Blocks.Sources.RealExpression optVar1
    annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
  IDEAS.Buildings.Components.BoundaryWall boundaryWall(
    inc=IDEAS.Types.Tilt.Floor,
    azi=rectangularZoneTemplate.aziA,
    A=rectangularZoneTemplate.A,
    redeclare Data.HeavyFloorTABS constructionType)
                                 annotation (Placement(transformation(
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
    dp_nominal=0,
    m_flow_nominal=0.5)
    annotation (Placement(transformation(extent={{6,-70},{26,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_tabs(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04) "Pressure source"
    annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
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
  Modelica.Blocks.Sources.RealExpression optVar2
    annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,-50})));
  BuildingMpc.Fluid.HeatPumps.HeatPump heatPump(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    dp2_nominal=0,
    dp1_nominal=0,
    Q_nom=5000,
    m1_flow_nominal=0.5,
    m2_flow_nominal=1.2,
    COP_expr=4.5)
    annotation (Placement(transformation(extent={{88,-44},{108,-64}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(
    filDat=IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(),
    soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(),
    conDat=IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example())
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume vol(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=0.5,
    V=1,
    nPorts=4,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    T_start=308.15)
    annotation (Placement(transformation(extent={{54,-100},{74,-80}})));
  Modelica.Blocks.Sources.RealExpression TMeas(y=0.5)
    annotation (Placement(transformation(extent={{74,-98},{94,-78}})));
  Modelica.Blocks.Sources.RealExpression optVar3
    annotation (Placement(transformation(extent={{24,-36},{44,-16}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_bor_in(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=1.2,
    tau=0) annotation (Placement(transformation(
        extent={{6.5,-8.5},{-6.5,8.5}},
        rotation=-90,
        origin={65.5,-0.5})));
  Fluid.Geothermal.Borefields.OneUTube             multipleBorehole(
    redeclare package Medium = IDEAS.Media.Water,
    borFieDat=borFieDat,
    TExt0_start=(273.15 + 13.5),
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dynFil=true)
    annotation (Placement(transformation(extent={{44,20},{24,40}})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
        points={{16,-50},{16,-38},{2,-38}},          color={191,0,0}));
  connect(m_flow_tabs.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-24,-60},{6,-60}},  color={0,127,255}));
  connect(optVar1.y,m_flow_tabs. m_flow_in) annotation (Line(points={{-65,-10},
          {-40,-10},{-40,-52},{-40,-52}}, color={0,0,127}));
  connect(optVar2.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
          {-22,90},{44,90},{44,80}},        color={0,0,127}));
  connect(sink.ports[1],m_flow_tabs. port_a) annotation (Line(points={{-78,-60},
          {-61,-60},{-44,-60}}, color={0,127,255}));
  connect(m_flow_source.port_b, heatPump.port_a2) annotation (Line(points={{60,72},
          {108,72},{108,-48}},                color={0,127,255}));
  connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
          {-26,72},{-26,72},{40,72}}, color={0,127,255}));
  connect(boundaryWall.propsBus_a, rectangularZoneTemplate.proBusFlo[1])
    annotation (Line(
      points={{-10,-33},{-10,-16}},
      color={255,204,51},
      thickness=0.5));
  connect(heatPump.port_a1, vol.ports[1]) annotation (Line(points={{88,-60},{64,
          -60},{64,-100},{61,-100}}, color={0,127,255}));
  connect(m_flow_sink.port_b, vol.ports[2]) annotation (Line(points={{136,-106},
          {63,-106},{63,-100}}, color={0,127,255}));
  connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{156,
          -106},{156,-60},{108,-60}},           color={0,127,255}));
  connect(TMeas.y, m_flow_sink.m_flow_in) annotation (Line(points={{95,-88},{
          152,-88},{152,-98}},                  color={0,0,127}));
  connect(optVar3.y, heatPump.Tcon_out) annotation (Line(points={{45,-26},{56,
          -26},{56,-63},{88,-63}}, color={0,0,127}));
  connect(T_bor_in.port_b, multipleBorehole.port_a) annotation (Line(points={{
          65.5,6},{66,6},{66,30},{44,30}}, color={0,127,255}));
  connect(multipleBorehole.port_b, m_flow_source.port_a) annotation (Line(
        points={{24,30},{20,30},{20,72},{40,72}}, color={0,127,255}));
  connect(T_bor_in.port_a, heatPump.port_b2) annotation (Line(points={{65.5,-7},
          {65.5,-48},{88,-48}}, color={0,127,255}));
  connect(vol.ports[3], embeddedPipe.port_b) annotation (Line(points={{65,-100},
          {44,-100},{44,-60},{26,-60}}, color={0,127,255}));
  connect(vol.ports[4], m_flow_tabs.port_a) annotation (Line(points={{67,-100},
          {62,-100},{62,-106},{-52,-106},{-52,-60},{-44,-60}}, color={0,127,255}));
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
end Case900GEOTABS_bufferTank;
