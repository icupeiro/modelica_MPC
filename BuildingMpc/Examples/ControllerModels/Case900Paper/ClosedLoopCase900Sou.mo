within BuildingMpc.Examples.ControllerModels.Case900Paper;
model ClosedLoopCase900Sou
  extends Case900Sou(
    u1 = mpc.u1,
    u2 = mpc.u2);
  MpcCase900Sou mpc(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect)
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));

   Real[38] states;

    model MpcCase900Sou
    extends UnitTests.MPC.BaseClasses.Mpc(
      final nOut=6,
      final nOpt=2,
      final nSta=38,
      final nMeas=0,
      final nIneq=8,
      final nLLIn=0,
      final nLLOut=0,
      final nLLSta=0,
      final horizonLength=24,
      final numControlIntervals=12,
      final controlTimeStep=3600,
      final nModCorCoeff=33,
      final name= "Case900Sou");
    Modelica.Blocks.Interfaces.RealOutput Q_cond = getOutput(tableID,3, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput TRad = getOutput(tableID,4, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,1, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,2, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,6, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,5, time)
      annotation (Placement(transformation(extent={{96,50},{116,70}})));
    end MpcCase900Sou;

equation
  states = mpc.uSta;

  states = {
  rectangularZoneTemplate.winA.heaCapGla.T,
rectangularZoneTemplate.winB.heaCapGla.T,
rectangularZoneTemplate.winC.heaCapGla.T,
rectangularZoneTemplate.winD.heaCapGla.T,
rectangularZoneTemplate.bouFlo.layMul.port_gain[1].T,
rectangularZoneTemplate.bouFlo.layMul.port_b.T,
embeddedPipe.heatPortEmb[1].T,
rectangularZoneTemplate.propsBusInt[5].surfRad.T,
rectangularZoneTemplate.outA.port_emb[1].T,
rectangularZoneTemplate.outA.extCon.port_a.T,
rectangularZoneTemplate.outA.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[1].surfRad.T,
rectangularZoneTemplate.outA.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outB.port_emb[1].T,
rectangularZoneTemplate.outB.extCon.port_a.T,
rectangularZoneTemplate.outB.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[2].surfRad.T,
rectangularZoneTemplate.outB.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outC.port_emb[1].T,
rectangularZoneTemplate.outC.extCon.port_a.T,
rectangularZoneTemplate.outC.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[3].surfRad.T,
rectangularZoneTemplate.outC.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outD.port_emb[1].T,
rectangularZoneTemplate.outD.extCon.port_a.T,
rectangularZoneTemplate.outD.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[4].surfRad.T,
rectangularZoneTemplate.outD.layMul.monLay[3].monLayDyn.T[2],
rectangularZoneTemplate.outCei.port_emb[1].T,
rectangularZoneTemplate.outCei.extCon.port_a.T,
rectangularZoneTemplate.outCei.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[6].surfRad.T,
rectangularZoneTemplate.int.port_emb[1].T,
rectangularZoneTemplate.propsBusInt[8].surfRad.T,
rectangularZoneTemplate.int.layMul.port_gain[2].T,
rectangularZoneTemplate.propsBusInt[7].surfRad.T,
rectangularZoneTemplate.radDistr.TRad,
rectangularZoneTemplate.airModel.vol.T};

end ClosedLoopCase900Sou;
