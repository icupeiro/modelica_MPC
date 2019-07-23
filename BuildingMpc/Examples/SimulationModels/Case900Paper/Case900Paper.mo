within BuildingMpc.Examples.SimulationModels.Case900Paper;
model Case900Paper
  extends Modelica.Icons.Example;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=278.15, X_a=0.25);

  IBPSA.Fluid.Sources.Boundary_pT bou(          redeclare package Medium =
        IDEAS.Media.Air, nPorts=1)
    annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
  inner IDEAS.Buildings.Validation.BaseClasses.SimInfoManagerBestest
                                          sim(Tdes=273.15 - 21)
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Modelica.Blocks.Sources.RealExpression optVar2(y=0)
    annotation (Placement(transformation(extent={{40,-6},{20,14}})));
  IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = Glycol,
    dp1_nominal=10000,
    dp2_nominal=10000,
    redeclare package ref = IDEAS.Media.Refrigerants.R410A,
    enable_temperature_protection=true,
    datHeaPum=
        IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    scaling_factor=0.126,
    T2_start=borFie.TExt0_start,
    T1_start=298.15,
    TEvaMin=273.151)                              annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={62,-30})));
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow airSystem
    annotation (Placement(transformation(extent={{14,-6},{-6,14}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    A_floor=rectangularZoneTemplate.A,
    dp_nominal=0,
    computeFlowResistance=false,
    T_start(displayUnit="K") = 298.15,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-28,-46},{-8,-26}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    nPorts=1,
    p=200000,
    T=273.25) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-84,-20})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(
    filDat=IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(),
    soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
        kSoi=1.4,
        cSoi=980,
        dSoi=1358.4),
    conDat=IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example(
        mBor_flow_nominal=0.1,
        hBor=64.5,
        dBor=0,
        nBor=1,
        cooBor={{0,0}},
        xC=0.04))
    annotation (Placement(transformation(extent={{-100,-80},{-80,-60}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow tabs_pump(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{30,-88},{10,-68}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow borFie_pump(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    redeclare package Medium = Glycol,
    m_flow_nominal=0.1,
    T_start=borFie.TExt0_start)
    annotation (Placement(transformation(extent={{52,72},{72,92}})));
public
  IBPSA.Fluid.Sources.Boundary_pT source(
    p=200000,
    nPorts=1,
    redeclare package Medium = Glycol)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
  IBPSA.Fluid.Geothermal.Borefields.OneUTube borFie(
                           borFieDat=borFieDat, redeclare package Medium =
        Glycol,
    r={6},
    TExt0_start=283.15,
    nbTem=1)
    annotation (Placement(transformation(extent={{0,70},{20,90}})));
  Modelica.Blocks.Sources.RealExpression optVar1(y=0)
    annotation (Placement(transformation(extent={{-8,-30},{12,-10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TSup(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    m_flow_nominal=0.05,
    T_start=298.15)
    annotation (Placement(transformation(extent={{-58,-26},{-38,-46}})));
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow auxHeaSystem
    annotation (Placement(transformation(extent={{12,22},{-8,42}})));
  IDEAS.Controls.Continuous.LimPID conPID1(
    yMin=0,
    Ti=30,
    controllerType=Modelica.Blocks.Types.SimpleController.P,
    yMax=1225,
    k=1225)
    annotation (Placement(transformation(extent={{118,42},{98,22}})));

  Modelica.Blocks.Sources.Constant mFlow(k=0.1)
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir airModel(
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState),
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.BoundaryWall,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypA,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypB,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypC,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypD,
    hasWinA=true,
    fracA=0,
    redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
      shaTypA,
    bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    aziA=IDEAS.Types.Azimuth.S,
    l=8,
    w=6,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingA,
    hasWinB=true,
    hasWinC=true,
    hasWinD=true,
    A_winA=3,
    A_winB=3,
    A_winC=3,
    A_winD=3,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloorFH
      conTypFlo,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingB,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingC,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingD,
    hasInt=true,
    lInt=10,
    redeclare IDEAS.Examples.PPD12.Data.InteriorWall10 conTypInt,
    hasEmb=true,
    fracB=0,
    fracC=0,
    fracD=0,
    redeclare package Medium = IDEAS.Media.Air,
    mSenFac=0.822,
    T_start=296.15)
    annotation (Placement(transformation(extent={{-50,-14},{-30,6}})));

  IDEAS.Fluid.Sensors.TemperatureTwoPort TRet(
    tau=0,
    redeclare package Medium = Glycol,
    m_flow_nominal=0.1,
    T_start=borFie.TExt0_start)
    annotation (Placement(transformation(extent={{26,70},{46,90}})));
  Modelica.Blocks.Sources.RealExpression nightSetBack(y=if (clock.hour >= 7
         and clock.hour <= 23) then 273.15 + 21 else 273.15 + 16)
    "constraint with night set-back"
    annotation (Placement(transformation(extent={{148,30},{128,50}})));
  UnitTests.Components.Clock clock
    annotation (Placement(transformation(extent={{-100,-4},{-80,16}})));
  Modelica.Blocks.Interfaces.RealOutput energyUse
    annotation (Placement(transformation(extent={{160,-72},{180,-52}})));
  Modelica.Blocks.Continuous.Integrator totalElecTokWh(k=1/3600/1000)
    annotation (Placement(transformation(extent={{102,-94},{122,-74}})));
  Modelica.Blocks.Continuous.Integrator totalGasTokWh(k=1/3600/1000)
    annotation (Placement(transformation(extent={{102,-50},{122,-30}})));
  Modelica.Blocks.Math.Add totalGas
    annotation (Placement(transformation(extent={{58,-8},{78,12}})));
  Modelica.Blocks.Math.Add totalEnergy
    annotation (Placement(transformation(extent={{134,-72},{154,-52}})));
  Modelica.Blocks.Interfaces.RealOutput operationalCosts
    annotation (Placement(transformation(extent={{160,-14},{180,6}})));
  Modelica.Blocks.Math.Add totalCost
    annotation (Placement(transformation(extent={{136,-14},{156,6}})));
  Modelica.Blocks.Math.Gain gasCost(k=gasPrice.k)
                                             "in EUR/kWh"
    annotation (Placement(transformation(extent={{114,-4},{126,8}})));
  Modelica.Blocks.Math.Gain elecCost(k=electricityPrice.k)
                                            "in EUR/kWh"
    annotation (Placement(transformation(extent={{114,-16},{126,-4}})));
  Modelica.Blocks.Sources.Constant gasPrice(k=0.061)
    annotation (Placement(transformation(extent={{-100,-96},{-90,-86}})));
  Modelica.Blocks.Sources.Constant electricityPrice(k=0.204)
    annotation (Placement(transformation(extent={{-84,-96},{-74,-86}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TCon(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    m_flow_nominal=0.05,
    T_start=298.15)
    annotation (Placement(transformation(extent={{14,-26},{34,-46}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    tau=0,
    redeclare package Medium = Glycol,
    m_flow_nominal=0.1,
    T_start=borFie.TExt0_start)
    annotation (Placement(transformation(extent={{10,42},{-10,62}})));
  Modelica.Blocks.Sources.RealExpression optVar3(y=0)
    annotation (Placement(transformation(extent={{-32,84},{-12,104}})));
  Modelica.Blocks.Continuous.Integrator geoTokWh(k=1/3600/1000)
    annotation (Placement(transformation(extent={{98,66},{118,86}})));
  Modelica.Blocks.Continuous.Integrator FhTokWh(k=1/3600/1000)
    annotation (Placement(transformation(extent={{56,-72},{76,-52}})));
equation
  connect(airSystem.Q_flow, optVar2.y)
    annotation (Line(points={{14,4},{19,4}}, color={0,0,127}));
  connect(tabs_pump.port_a, heaPum.port_b1) annotation (Line(points={{30,-78},{86,
          -78},{86,-36},{72,-36}}, color={0,127,255}));
  connect(borFie_pump.port_b, heaPum.port_a2) annotation (Line(points={{72,82},
          {86,82},{86,-24},{72,-24}},color={0,127,255}));
  connect(source.ports[1], borFie.port_a) annotation (Line(points={{-26,70},{-14,
          70},{-14,80},{0,80}}, color={0,127,255}));
  connect(TSup.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-38,-36},{-28,-36}}, color={0,127,255}));
  connect(TSup.port_a, tabs_pump.port_b) annotation (Line(points={{-58,-36},{-68,
          -36},{-68,-78},{10,-78}}, color={0,127,255}));
  connect(sink.ports[1], tabs_pump.port_b) annotation (Line(points={{-84,-30},{-84,
          -36},{-68,-36},{-68,-78},{10,-78}}, color={0,127,255}));
  connect(conPID1.y, auxHeaSystem.Q_flow)
    annotation (Line(points={{97,32},{12,32}}, color={0,0,127}));
  connect(mFlow.y, tabs_pump.m_flow_in) annotation (Line(points={{-79,50},{-70,
          50},{-70,-14},{-60,-14},{-60,-60},{20,-60},{20,-66}}, color={0,0,127}));
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-50,40},{-38,40},{-38,6}}, color={0,127,255}));
  connect(auxHeaSystem.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{-8,32},{-20,32},{-20,-7},{-30,-7}}, color={191,0,0}));
  connect(airSystem.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{-6,4},{-18,4},{-18,-7},{-30,-7}}, color={191,0,0}));
  for i in 1:embeddedPipe.nDiscr loop
  connect(embeddedPipe.heatPortEmb[i], rectangularZoneTemplate.gainEmb[1])    annotation (Line(points={{-18,-26},{-18,-13},{-30,-13}}, color={191,0,0}));
  end for;
  connect(borFie.port_b, TRet.port_a)
    annotation (Line(points={{20,80},{26,80}}, color={0,127,255}));
  connect(TRet.port_b, borFie_pump.port_a)
    annotation (Line(points={{46,80},{50,80},{50,82},{52,82}},
                                               color={0,127,255}));
  connect(nightSetBack.y, conPID1.u_s) annotation (Line(points={{127,40},{124,
          40},{124,32},{120,32}}, color={0,0,127}));
  connect(heaPum.P, totalElecTokWh.u) annotation (Line(points={{73,-30},{88,-30},
          {88,-84},{100,-84}}, color={0,0,127}));
  connect(optVar2.y, totalGas.u2)
    annotation (Line(points={{19,4},{20,4},{20,-4},{56,-4}}, color={0,0,127}));
  connect(conPID1.y, totalGas.u1)
    annotation (Line(points={{97,32},{52,32},{52,8},{56,8}}, color={0,0,127}));
  connect(totalGas.y, totalGasTokWh.u) annotation (Line(points={{79,2},{90,2},{
          90,-40},{100,-40}}, color={0,0,127}));
  connect(totalGasTokWh.y, totalEnergy.u1) annotation (Line(points={{123,-40},{
          130,-40},{130,-56},{132,-56}}, color={0,0,127}));
  connect(totalElecTokWh.y, totalEnergy.u2)
    annotation (Line(points={{123,-84},{132,-84},{132,-68}}, color={0,0,127}));
  connect(totalEnergy.y, energyUse)
    annotation (Line(points={{155,-62},{170,-62}}, color={0,0,127}));
  connect(totalCost.y, operationalCosts)
    annotation (Line(points={{157,-4},{170,-4}}, color={0,0,127}));
  connect(gasCost.y, totalCost.u1)
    annotation (Line(points={{126.6,2},{134,2}}, color={0,0,127}));
  connect(elecCost.y, totalCost.u2)
    annotation (Line(points={{126.6,-10},{134,-10}}, color={0,0,127}));
  connect(totalGasTokWh.y, gasCost.u) annotation (Line(points={{123,-40},{128,
          -40},{128,-22},{102,-22},{102,2},{112.8,2}}, color={0,0,127}));
  connect(totalElecTokWh.y, elecCost.u) annotation (Line(points={{123,-84},{126,
          -84},{126,-20},{108,-20},{108,-10},{112.8,-10}}, color={0,0,127}));
  connect(optVar1.y, heaPum.y) annotation (Line(points={{13,-20},{32,-20},{32,
          -33},{50,-33}}, color={0,0,127}));
  connect(rectangularZoneTemplate.TSensor, conPID1.u_m) annotation (Line(points=
         {{-29,-2},{-24,-2},{-24,44},{108,44}}, color={0,0,127}));
  connect(embeddedPipe.port_b, TCon.port_a)
    annotation (Line(points={{-8,-36},{14,-36}}, color={0,127,255}));
  connect(TCon.port_b, heaPum.port_a1)
    annotation (Line(points={{34,-36},{52,-36}}, color={0,127,255}));
  connect(heaPum.port_b2, TIn.port_a) annotation (Line(points={{52,-24},{46,-24},
          {46,52},{10,52}}, color={0,127,255}));
  connect(TIn.port_b, borFie.port_a) annotation (Line(points={{-10,52},{-14,52},
          {-14,80},{0,80}}, color={0,127,255}));
  connect(optVar3.y, borFie_pump.m_flow_in) annotation (Line(points={{-11,94},{
          62,94}},                 color={0,0,127}));
  connect(heaPum.QEva_flow, geoTokWh.u) annotation (Line(points={{73,-21},{84,
          -21},{84,76},{96,76}}, color={0,0,127}));
  connect(heaPum.QCon_flow, FhTokWh.u) annotation (Line(points={{73,-39},{82,
          -39},{82,-46},{48,-46},{48,-62},{54,-62}}, color={0,0,127}));
 annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {160,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{160,100}})),
    experiment(
      StopTime=5259487,
      Interval=300,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
      Evaluate=false,
      OutputCPUtime=true,
      OutputFlatModelica=false));
end Case900Paper;
