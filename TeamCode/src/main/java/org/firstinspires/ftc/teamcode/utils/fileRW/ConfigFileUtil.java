package org.firstinspires.ftc.teamcode.utils.fileRW;

import android.content.Context;
import android.content.res.Resources;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.PrintWriter;
import java.net.URL;
import java.util.List;

/**
 * Utility functions for log files.
 */
public class ConfigFileUtil {
    public static final String dir = ConfigFileUtil.class.getResource("/").getFile();;
    public static final String fileFormat = ".virus";

    public enum ConfigDataType{
        INT,
        FLOAT,
        STRING,
        DOUBLE,
        BYTE,
        LONG,
    }

    //private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now

    public static void writeToConfig(String name, List<?> data, int rows) {
        try {

            PrintWriter writer = new PrintWriter( dir + name + fileFormat, "UTF-8");
            BufferedWriter bufferedWriter = new BufferedWriter(writer);
            for (int i = 0; i < rows; i++) {
                String line = "";
                for (int j = 0; j < data.size()/rows; j++) {
                    line += data.get(i * rows + j).toString() + ",";
                }

                bufferedWriter.write(line);
                bufferedWriter.newLine();
            }

            bufferedWriter.flush();
            bufferedWriter.close();

        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static File getConfig(String name){
        return new File(dir + name + fileFormat);
    }

    public static void readConfig(String name, Object[][] target, ConfigDataType type){
        int rows = target.length;
        String line;
        String[] data;

        URL resources = ConfigFileUtil.class.getResource(name);

        try{
            FileReader fileReader = new FileReader(new File(resources.toURI()));
            BufferedReader bufferedReader = new BufferedReader(fileReader);

            for (int i = 0; i < rows; i++) {
                if ((line = bufferedReader.readLine()) != null){
                    data = line.split(",");

                    for (int j = 0; j < data.length; j++){
                        switch (type){
                            case INT:
                                target[i][j] = Integer.parseInt(data[j]);
                                break;
                            case BYTE:
                                target[i][j] = Byte.parseByte(data[j]);
                                break;
                            case FLOAT:
                                target[i][j] = Float.parseFloat(data[j]);
                                break;
                            case LONG:
                                target[i][j] = Long.parseLong(data[j]);
                                break;
                            case DOUBLE:
                                target[i][j] = Double.parseDouble(data[j]);
                                break;
                            case STRING:
                                target[i][j] = data[j];
                                break;
                        }
                    }

                }
            }

        } catch (Exception e){
            e.printStackTrace();
        }
    }
}
