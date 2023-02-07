package org.jmhsrobotics.frc2023.util.sim;

import java.lang.reflect.Field;
import java.lang.reflect.Type;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.jmhsrobotics.frc2023.BuildConstants;

// TODO: Add Javadocs
public class BuildDataLogger {

    public static void LogToWpiLib(DataLog log) {
        try {
            String entry = "BuildInfo/";
            for (Field field : BuildConstants.class.getFields()) {
                String name = field.getName();
                Type t = field.getType();
                if (t.equals(int.class) || t.equals(long.class) || t.equals(double.class)) {
                    // System.out.println(name + " " + field.get(null));
                    var tmp = new DoubleLogEntry(log, entry + name);
                    tmp.append(field.getDouble(null));
                    tmp.finish();
                } else {
                    // System.out.println(name + " " + field.get(null));
                    var tmp = new StringLogEntry(log, entry + name);
                    
                   
                    if (field.get(null) != null) {
                        tmp.append(field.get(null).toString());
                    } else {
                        tmp.append(null);
                    }
                    tmp.finish();

                }
            }
        } catch (IllegalArgumentException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }

    public static void LogToNetworkTables() {
        try {
            String entry = "BuildInfo/";
            for (Field field : BuildConstants.class.getFields()) {
                String name = field.getName();
                Type t = field.getType();
                if (t.equals(int.class) || t.equals(long.class) || t.equals(double.class)) {
                    // System.out.println(name + " " + field.get(null));
                    SmartDashboard.putNumber(entry + name, field.getDouble(null));
                } else {
                    // System.out.println(name + " " + field.get(null));
                    if (field.get(null) != null) {
                        SmartDashboard.putString(entry + name, field.get(null).toString());
                    } else {
                        SmartDashboard.putString(entry + name, null);
                    }

                }
            }
        } catch (IllegalArgumentException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        } catch (IllegalAccessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }

}
