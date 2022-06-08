package org.firstinspires.ftc.teamcode.utils.fileRW.main;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.lexer.LexerTemplate;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParsedFile;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParserTemplate;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WritableElement;
import org.firstinspires.ftc.teamcode.utils.fileRW.main.writer.WriterTemplate;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.Vector;

public class FileRW{

    public static final String defaultDir = Environment.getExternalStorageDirectory().getPath() + "/UD_ROBOT_CONFIG/";

    private String root_directory;
    private boolean initialized = false;

    private LexerTemplate lexer;
    private ParserTemplate parser;
    private WriterTemplate writer;

    //explicitly removes empty constructor
    private FileRW(){}

    public FileRW(String root_directory, LexerTemplate lexerBehavior, ParserTemplate parserBehavior, WriterTemplate writeBehavior){
        this.root_directory = root_directory;
        this.lexer = lexerBehavior;
        this.parser = parserBehavior;
        this.writer = writeBehavior;
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

    public void finalizeWriteTo(String filepath){
        File writable = new File(root_directory + filepath);

        if (!initialized) throw new RuntimeException("This class hasn't been initialized yet!");

        try{
            BufferedWriter bw = new BufferedWriter(new FileWriter(writable));
            List<String> strings = writer.getBufferAsStringRows();

            for (String s: strings) {
                bw.write(s);
                bw.newLine();
            }

            bw.flush();

        } catch (Exception e){
            e.printStackTrace();
        }
    }

    //Wrapper methods for handling the write buffer

    public ParsedFile parse(Vector<FileRWRow> rows) throws IOException {
        return parser.parse(lexer.lex(rows));
    }
    public void writeToRow(int row, List<WritableElement> writableElements){
        writer.writeToRow(row, writableElements);
    }

    public void writeToCoord(int x, int y, WritableElement writableElement){
        writer.writeToCoord(x, y, writableElement);
    }

    public void deleteElement(int x, int y){
        writer.deleteElement(x, y);
    }

    public void deleteRow(int row){
        writer.deleteRow(row);
    }

    public void deleteAll(){
        writer.deleteAll();
    }

    public void setBufferedElements(List<List<WritableElement>> nBufferedElements){
        writer.setBufferedElements(nBufferedElements);
    }

    public List<List<WritableElement>> getBufferedElements(){
        return writer.getBufferedElements();
    }
}
