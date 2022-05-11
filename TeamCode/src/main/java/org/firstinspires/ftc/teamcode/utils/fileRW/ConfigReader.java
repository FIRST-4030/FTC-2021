package org.firstinspires.ftc.teamcode.utils.fileRW;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.PrintWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility functions for log files.
 */
public class ConfigReader {
    public static final String dir = (new File(AppUtil.ROOT_FOLDER + "/Robot_ConfigUD/")).getName();
    public static final String fileFormat = ".virus";

    private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now

    public static void writeToConfig(String name, List<String> data) {
        try {

            PrintWriter writer = new PrintWriter( dir + name + fileFormat, "UTF-8");
            for (String line: data) {
                writer.println(line);
            }
            writer.close();

        } catch (Exception e){
            e.printStackTrace();
        }
    }

    public static File getConfig(String name){
        return new File(dir + name + fileFormat);
    }
}
