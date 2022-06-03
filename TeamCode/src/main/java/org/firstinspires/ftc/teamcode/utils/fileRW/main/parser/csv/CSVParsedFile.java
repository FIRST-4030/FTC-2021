package org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.csv;

import org.firstinspires.ftc.teamcode.utils.fileRW.main.parser.ParsedFile;

import java.util.Vector;

public class CSVParsedFile implements ParsedFile {

    private Vector<?>[] contentMap;

    public CSVParsedFile(){
        this.contentMap = null;
    }

    public void setContentMap(Vector<?>[] contentMap){
        this.contentMap = contentMap;
    }

    public Vector<?> getRow(int idx){
        if (!(idx < contentMap.length)) throw new ArrayIndexOutOfBoundsException("This row doesn't exist!");
        return this.contentMap[idx];
    }

    @Override
    public void delete() {
        for (Vector<?> row: contentMap) {
            row.clear();
        }
        contentMap = null;
    }

    @Override
    public Vector<?>[] getAll() {
        return this.contentMap;
    }
}
