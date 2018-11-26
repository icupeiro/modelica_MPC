within BuildingMpc.Fluid.Geothermal.Borefields.Linear;
model Linearisation
  extends Modelica.Icons.Example;
  replaceable package Medium =
      IDEAS.Media.Water "Medium"
      annotation (choicesAllMatching = true);
  parameter Modelica.SIunits.TemperatureDifference dT = 3 "design temperature difference in the borefield";
  Buses.InputBus inputBus annotation (Placement(transformation(extent={{-120,58},
            {-80,98}}), iconTransformation(extent={{-208,28},{-188,48}})));
  Buses.OutputBus outputBus
    annotation (Placement(transformation(extent={{-120,30},{-80,70}})));
  OneUTube oneUTube(
    borFieDat=borFieDat,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true,
    Tsoil=286.65,
    dp_nominal=borFieDat.conDat.dp_nominal)
    annotation (Placement(transformation(extent={{-30,-20},{10,20}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) annotation (Placement(transformation(extent={{20,-10},{40,10}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  IDEAS.Fluid.Sources.Boundary_ph bou(redeclare package Medium = Medium, nPorts=
       1) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-90,-18})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={60,30})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u
                                          outCon(
    final massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare final package Medium = Medium,
    final allowFlowReversal=false,
    final m_flow_nominal=borFieDat.conDat.mBorFie_flow_nominal,
    final tau=0,
    final energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    dp_nominal=borFieDat.conDat.dp_nominal,
    Q_flow_nominal=1)
                   "Model to set outlet conditions"
    annotation (Placement(transformation(extent={{8,50},{-18,76}})));
public
  Modelica.Blocks.Math.Gain gain(k=1/(Medium.cp_const*dT))
    annotation (Placement(transformation(extent={{-12,16},{8,36}})));
equation
  connect(TIn.port_b, oneUTube.port_a)
    annotation (Line(points={{-40,0},{-30,0}}, color={0,127,255}));
  connect(oneUTube.port_b, TOut.port_a)
    annotation (Line(points={{10,0},{20,0}}, color={0,127,255}));
  connect(TOut.port_b, mFlow.port_a)
    annotation (Line(points={{40,0},{60,0},{60,20}},
                                             color={0,127,255}));
  connect(TOut.T, outputBus.TBorOut) annotation (Line(points={{30,11},{30,50.1},
          {-99.9,50.1}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(bou.ports[1], TIn.port_a)
    annotation (Line(points={{-90,-8},{-90,0},{-60,0}}, color={0,127,255}));
  connect(mFlow.port_b, outCon.port_a)
    annotation (Line(points={{60,40},{60,63},{8,63}}, color={0,127,255}));
  connect(outCon.port_b, TIn.port_a)
    annotation (Line(points={{-18,63},{-60,63},{-60,0}}, color={0,127,255}));
  connect(gain.y, mFlow.m_flow_in) annotation (Line(points={{9,26},{36,26},{36,
          24},{52,24}}, color={0,0,127}));
  connect(outCon.Q_flow, gain.u) annotation (Line(points={{-19.3,70.8},{-40,
          70.8},{-40,26},{-14,26}}, color={0,0,127}));
  connect(outCon.u, inputBus.Qbor) annotation (Line(points={{10.6,70.8},{32,
          70.8},{32,80},{-108,80}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Linearisation;
