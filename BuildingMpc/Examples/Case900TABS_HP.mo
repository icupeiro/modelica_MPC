within BuildingMpc.Examples;
model Case900TABS_HP "MPC template based on bestest case 900"
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
    useFluPor=true,
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
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
      constructionType,
    inc=IDEAS.Types.Tilt.Floor,
    azi=rectangularZoneTemplate.aziA,
    A=rectangularZoneTemplate.A) annotation (Placement(transformation(
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
    m_flow_nominal=5,
    A_floor=rectangularZoneTemplate.A,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial)
    annotation (Placement(transformation(extent={{16,-70},{36,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT source(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    T=287.15,
    nPorts=2)      annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
  Modelica.Blocks.Sources.RealExpression optVar2
    annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04) "Pressure source"
    annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
public
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u HP_eva(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    dp_nominal=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    Q_flow_nominal(displayUnit="kW") = 5000)
    annotation (Placement(transformation(extent={{60,62},{80,82}})));
  IDEAS.Fluid.HeatExchangers.PrescribedOutlet HP_con(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    dp_nominal=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_TSet=true,
    use_X_wSet=false)
    annotation (Placement(transformation(extent={{-78,-70},{-58,-50}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
    redeclare final package Medium = IDEAS.Media.Water,
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-4) "Pressure source"
    annotation (Placement(transformation(extent={{0,62},{20,82}})));
public
  Modelica.Blocks.Sources.RealExpression optVar3
    annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
  Modelica.Blocks.Sources.RealExpression Q_eva(y=-HP_con.Q_flow*COP.y)
    annotation (Placement(transformation(extent={{30,80},{50,100}})));
  IBPSA.Fluid.Sources.Boundary_pT bouW(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    p=200000,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={44,-26})));
  Modelica.Blocks.Sources.RealExpression COP(y=6.4 - 0.16*(T_con.T - 298.15) + 0.1
        *(T_eva.T - 288.15))
    annotation (Placement(transformation(extent={{26,10},{46,30}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{30,62},{50,82}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{10,-94},{-10,-74}})));
equation
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
  connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
    annotation (Line(
      points={{-10,-16},{-10,-33}},
      color={255,204,51},
      thickness=0.5));
  connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
        points={{26,-50},{26,-50},{26,-38},{2,-38}}, color={191,0,0}));
  connect(m_flow_sink.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-24,-60},{16,-60}}, color={0,127,255}));
  connect(optVar1.y, m_flow_sink.m_flow_in) annotation (Line(points={{-65,-10},
          {-40,-10},{-40,-52},{-40,-52}}, color={0,0,127}));
  connect(HP_con.port_b, m_flow_sink.port_a)
    annotation (Line(points={{-58,-60},{-44,-60}}, color={0,127,255}));
  connect(optVar2.y, HP_con.TSet) annotation (Line(points={{-65,-24},{-56,-24},
          {-56,-40},{-92,-40},{-92,-52},{-80,-52}}, color={0,0,127}));
  connect(source.ports[1], m_flow_source.port_a)
    annotation (Line(points={{-26,72},{0,72}}, color={0,127,255}));
  connect(HP_eva.port_b, source.ports[2]) annotation (Line(points={{80,72},{88,
          72},{88,50},{-22,50},{-22,68},{-26,68}}, color={0,127,255}));
  connect(optVar3.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
          {-22,90},{-22,90},{4,90},{4,80}}, color={0,0,127}));
  connect(Q_eva.y, HP_eva.u) annotation (Line(points={{51,90},{54,90},{54,78},{
          58,78}}, color={0,0,127}));
  connect(T_con.port_a, embeddedPipe.port_b) annotation (Line(points={{10,-84},{
          58,-84},{58,-60},{36,-60}}, color={0,127,255}));
  connect(T_con.port_b, HP_con.port_a) annotation (Line(points={{-10,-84},{-92,-84},
          {-92,-60},{-78,-60}}, color={0,127,255}));
  connect(m_flow_source.port_b, T_eva.port_a)
    annotation (Line(points={{20,72},{30,72}}, color={0,127,255}));
  connect(HP_eva.port_a, T_eva.port_b)
    annotation (Line(points={{60,72},{50,72}}, color={0,127,255}));
  connect(bouW.ports[1], embeddedPipe.port_b) annotation (Line(points={{54,-26},
          {58,-26},{58,-60},{36,-60}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StartTime=1420000000,
      StopTime=1430000000,
      __Dymola_fixedstepsize=1,
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
        "Simulate and plot"));
end Case900TABS_HP;
