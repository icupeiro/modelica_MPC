within BuildingMpc.Examples.ControllerModels;
model Case900GEOTABS_cooling_noBuffTank
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
  Modelica.Blocks.Sources.RealExpression optVar1
    annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
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
    dp_nominal=0,
    m_flow_nominal=0.5,
    Q(nominal=3000))
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
    final m_flow_small=1e-04,
    port_b(h_outflow(nominal=100000)))
                              "Pressure source"
    annotation (Placement(transformation(extent={{-4,-110},{-24,-90}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-4) "Pressure source"
    annotation (Placement(transformation(extent={{40,62},{60,82}})));
public
  Modelica.Blocks.Sources.RealExpression optVar2
    annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    p=200000,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-78,-50})));
  BuildingMpc.Fluid.HeatPumps.HeatPump heatPump(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = IDEAS.Media.Water,
    dp2_nominal=0,
    dp1_nominal=0,
    m1_flow_nominal=0.5,
    m2_flow_nominal=0.5,
    Q_nom=3000)
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
  Modelica.Blocks.Sources.RealExpression optVar3
    annotation (Placement(transformation(extent={{24,-36},{44,-16}})));
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
  Modelica.Blocks.Sources.RealExpression optVar4
    annotation (Placement(transformation(extent={{-90,-94},{-70,-74}})));
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
    m_flow_nominal=0.5*ones(3),
    dp_nominal=0*ones(3))
    annotation (Placement(transformation(extent={{58,-68},{74,-52}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveMotor(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    l=0,
    m_flow_nominal=0.5,
    allowFlowReversal=false)
    annotation (Placement(transformation(extent={{30,-110},{10,-90}})));
  IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor threeWayValveMotor1(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    l=0,
    allowFlowReversal=true,
    m_flow_nominal=1.2)
    annotation (Placement(transformation(extent={{10,-10},{-10,10}},
        rotation=90,
        origin={66,-36})));
  IDEAS.Fluid.FixedResistances.Junction jun1(
    redeclare package Medium = IDEAS.Media.Water,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    portFlowDirection_1=Modelica.Fluid.Types.PortFlowDirection.Entering,
    portFlowDirection_2=Modelica.Fluid.Types.PortFlowDirection.Leaving,
    dp_nominal=0*ones(3),
    portFlowDirection_3=Modelica.Fluid.Types.PortFlowDirection.Entering,
    m_flow_nominal=1.2*ones(3),
    res1(port_b(h_outflow(nominal=46300))))
    annotation (Placement(transformation(extent={{-8,-8},{8,8}},
        rotation=90,
        origin={66,-4})));
  Fluid.Geothermal.Borefields.OneUTube             multipleBorehole(
    redeclare package Medium = IDEAS.Media.Water,
    borFieDat=borFieDat,
    energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
    dynFil=true,
    TExt0_start=(273.15 + 10.8))
    annotation (Placement(transformation(extent={{44,20},{24,40}})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
        points={{16,-50},{16,-38},{2,-38}},          color={191,0,0}));
  connect(optVar1.y,m_flow_tabs. m_flow_in) annotation (Line(points={{-65,-10},
          {-40,-10},{-40,-92},{-8,-92}},  color={0,0,127}));
  connect(optVar2.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
          {-22,90},{44,90},{44,80}},        color={0,0,127}));
  connect(m_flow_source.port_b, heatPump.port_a2) annotation (Line(points={{60,72},
          {106,72},{106,-48}},                color={0,127,255}));
  connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
          {-26,72},{-26,72},{40,72}}, color={0,127,255}));
  connect(boundaryWall.propsBus_a, rectangularZoneTemplate.proBusFlo[1])
    annotation (Line(
      points={{-10,-33},{-10,-16}},
      color={255,204,51},
      thickness=0.5));
  connect(optVar3.y, heatPump.Tcon_out) annotation (Line(points={{45,-26},{56,
          -26},{56,-63},{86,-63}}, color={0,0,127}));
  connect(hex.port_b2, T_PC_sup.port_a) annotation (Line(points={{96,-34},{96,-38.5},
          {141,-38.5}},        color={0,127,255}));
  connect(jun.port_1, embeddedPipe.port_b)
    annotation (Line(points={{58,-60},{26,-60}}, color={0,127,255}));
  connect(jun.port_3, hex.port_a2) annotation (Line(points={{66,-68},{66,-136},
          {178,-136},{178,-14},{96,-14}}, color={0,127,255}));
  connect(sink.ports[1], embeddedPipe.port_a)
    annotation (Line(points={{-78,-60},{6,-60}}, color={0,127,255}));
  connect(threeWayValveMotor.port_b, m_flow_tabs.port_a)
    annotation (Line(points={{10,-100},{-4,-100}}, color={0,127,255}));
  connect(m_flow_tabs.port_b, embeddedPipe.port_a) annotation (Line(points={{
          -24,-100},{-32,-100},{-32,-60},{6,-60}}, color={0,127,255}));
  connect(threeWayValveMotor.port_a2, T_PC_sup.port_b) annotation (Line(points=
          {{20,-110},{20,-128},{168,-128},{168,-38.5},{154,-38.5}}, color={0,
          127,255}));
  connect(optVar4.y, threeWayValveMotor.ctrl) annotation (Line(points={{-69,-84},
          {20,-84},{20,-89.2}},                     color={0,0,127}));
  connect(heatPump.port_b2, threeWayValveMotor1.port_b) annotation (Line(points=
         {{86,-48},{76,-48},{76,-50},{66,-50},{66,-46}}, color={0,127,255}));
  connect(threeWayValveMotor1.port_a2, hex.port_a1) annotation (Line(points={{76,-36},
          {78,-36},{78,-34},{84,-34}},         color={0,127,255}));
  connect(jun1.port_2, T_bor_in.port_a) annotation (Line(points={{66,4},{66,8},
          {66,13},{65.5,13}}, color={0,127,255}));
  connect(jun1.port_1, threeWayValveMotor1.port_a1)
    annotation (Line(points={{66,-12},{66,-26}}, color={0,127,255}));
  connect(hex.port_b1, jun1.port_3) annotation (Line(points={{84,-14},{84,-4},{74,
          -4}},             color={0,127,255}));
  connect(threeWayValveMotor1.ctrl, optVar4.y) annotation (Line(points={{55.2,
          -36},{4,-36},{4,-38},{-48,-38},{-48,-84},{-69,-84}}, color={0,0,127}));
  connect(heatPump.port_a1, jun.port_2)
    annotation (Line(points={{86,-60},{74,-60}}, color={0,127,255}));
  connect(heatPump.port_b1, threeWayValveMotor.port_a1) annotation (Line(points=
         {{106,-60},{126,-60},{126,-100},{30,-100}}, color={0,127,255}));
  connect(m_flow_source.port_a, multipleBorehole.port_b) annotation (Line(
        points={{40,72},{34,72},{34,30},{24,30}}, color={0,127,255}));
  connect(multipleBorehole.port_a, T_bor_in.port_b) annotation (Line(points={{
          44,30},{56,30},{56,26},{65.5,26}}, color={0,127,255}));
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
end Case900GEOTABS_cooling_noBuffTank;
