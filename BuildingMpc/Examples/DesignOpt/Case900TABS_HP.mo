within BuildingMpc.Examples.DesignOpt;
model Case900TABS_HP
  "Controller model for the BESTEST Case900 with TABS and heat pump with an ideal source; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
  extends BuildingMpc.Examples.ControllerModels.Case900TABS_HP(heatPump(Q_nom=
          desVar.Power_HP), rectangularZoneTemplate(A_winA=desVar.fra_w_window));
  replaceable DecisionVariablesRecords.Case900TABS_HPDesignVariables desVar
    annotation (Placement(transformation(extent={{-86,16},{-66,36}})));
end Case900TABS_HP;
