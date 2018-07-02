within BuildingMpc.Examples.SimulationModels;
model Case900TABS "Case900 simulation model"
  extends IDEAS.Buildings.Validation.Cases.Case900(
    redeclare BuildingMpc.Examples.SimulationModels.Envelope.Bui900TABS building,
    redeclare HeatingSystems.Hea900TABS heatingSystem,
    redeclare IDEAS.Buildings.Validation.BaseClasses.Occupant.None occupant);
  Modelica.Blocks.Sources.RealExpression[30] states(y=
{(building.win[1].heaCapGla.T+building.win[2].heaCapGla.T)/2,
building.wall[3].port_emb[1].T,
building.wall[3].extCon.port_a.T,
building.wall[3].layMul.port_gain[2].T,
building.gF.propsBusInt[1].surfRad.T,
building.wall[3].layMul.monLay[3].monLayDyn.T[2],
building.wall[4].port_emb[1].T,
building.wall[4].extCon.port_a.T,
building.wall[4].layMul.port_gain[2].T,
building.gF.propsBusInt[2].surfRad.T,
building.wall[4].layMul.monLay[3].monLayDyn.T[2],
building.wall[1].port_emb[1].T,
building.wall[1].extCon.port_a.T,
building.wall[1].layMul.port_gain[2].T,
building.gF.propsBusInt[3].surfRad.T,
building.wall[1].layMul.monLay[3].monLayDyn.T[2],
building.wall[2].port_emb[1].T,
building.wall[2].extCon.port_a.T,
building.wall[2].layMul.port_gain[2].T,
building.gF.propsBusInt[4].surfRad.T,
building.wall[2].layMul.monLay[3].monLayDyn.T[2],
building.roof.port_emb[1].T,
building.roof.extCon.port_a.T,
building.roof.layMul.port_gain[2].T,
building.gF.propsBusInt[6].surfRad.T,
building.gF.airModel.vol.dynBal.U,
building.gF.gainRad.T,
building.floor.port_emb[1].T,
building.floor.layMul.port_b.T,
building.floor.intCon_a.port_a.T})
    annotation (Placement(transformation(extent={{-40,58},{-20,78}})));
equation
  connect(states.y, heatingSystem.u) annotation (Line(points={{-19,68},{-16,68},
          {-16,10},{-12.2,10}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=31536000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=20,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false));
end Case900TABS;
