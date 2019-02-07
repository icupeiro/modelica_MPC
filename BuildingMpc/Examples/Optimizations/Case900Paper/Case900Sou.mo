within BuildingMpc.Examples.Optimizations.Case900Paper;
model Case900Sou
  extends BuildingMpc.Examples.SimulationModels.Case900Paper.Case900Paper(optVar2(y=
         mpcCase900Sou.yOpt[2]), optVar1(y=mpcCase900Sou.yOpt[1]));

  Real states[36];

  MPCs.Case900Paper.MpcCase900Sou mpcCase900Sou(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect)
    annotation (Placement(transformation(extent={{-100,0},{-80,20}})));

equation
   states = {
rectangularZoneTemplate.winA.heaCapGla.T,
rectangularZoneTemplate.winB.heaCapGla.T,
rectangularZoneTemplate.winC.heaCapGla.T,
rectangularZoneTemplate.winD.heaCapGla.T,
embeddedPipe.heatPortEmb[1].T,
rectangularZoneTemplate.bouFlo.layMul.port_b.T,
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
rectangularZoneTemplate.airModel.vol.dynBal.U}
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));

        mpcCase900Sou.uSta = states;

end Case900Sou;
