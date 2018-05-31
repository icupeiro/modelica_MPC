within BuildingMpc.Examples.DesignOpt.DecisionVariablesRecords;
partial record Case900TABS_HPDesignVariables
  extends Modelica.Icons.Record;
  parameter Real fra_w_window(min=0,max=20) = w_window.val "Fraction of window width";
  parameter Real Power_HP(min=1000, max=5000) = P_HP.val "Max. Power of the HP";
  constant Integer depreciationYears = 25;
  parameter Templates.ContinuousDecisionVariable w_window(
    y_min=8,
    y_max=16,
    investment = w_window.val*200000,
    maintenance=0,
    lifetime = 30,
    replacement= 0,
    interest=0,
    depreciationYears=depreciationYears);
  parameter Templates.ContinuousDecisionVariable P_HP(
    y_min=1000,
    y_max=5000,
    investment = P_HP.val*850,
    maintenance=0.03*P_HP.val,
    lifetime = 20,
    replacement= 2,
    interest=0.02,
    depreciationYears=depreciationYears);
  parameter Real cost = P_HP.cost + w_window.cost;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Case900TABS_HPDesignVariables;
