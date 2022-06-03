package org.firstinspires.ftc.teamcode.utils.fileRW.main;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerTemplate;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParsedFile;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParserTemplate;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Vector;
import java.util.stream.Stream;

public class FileRW {

    public static final String defaultDir = Environment.getExternalStorageDirectory().getPath() + "/UD_ROBOT_CONFIG/";

    private String root_directory;
    private boolean initialized = false;


    //explicitly removes empty constructor
    public FileRW(String root_directory){
        this.root_directory = root_directory;
    }

    public void init(){
        File directory = new File(root_directory);
        if (!directory.exists()){
            directory.mkdir();
        }

        initialized = true;
    }

    /**
     * This method returns a Vector of FileRWRow for <br>
     * (1) Thread Safety <br>
     * (2) For further lexing then parsing
     * @param filepath
     * @return
     */
    public Vector<FileRWRow> read(String filepath) {
        File readable = new File(root_directory + filepath);

        if (!initialized) throw new RuntimeException("This class hasn't been initialized yet!");

        try {
            BufferedReader br = new BufferedReader(new FileReader(readable));

            Vector<FileRWRow> output = new Vector<>();

            String line = "";
            while (line != null){
                line = br.readLine();
                output.add(new FileRWRow(line));
            }

            br.close(); //explicitly call close method

            return output;
        } catch (Exception e){
            e.printStackTrace();
        }

        return null;
    }

    public ParsedFile parse(Vector<FileRWRow> rows, LexerTemplate lexer, ParserTemplate parser) throws IOException {
        return parser.parse(lexer.lex(rows));
    }
}
