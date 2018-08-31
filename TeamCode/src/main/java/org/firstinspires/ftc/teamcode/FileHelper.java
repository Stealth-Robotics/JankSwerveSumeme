package org.firstinspires.ftc.teamcode;

import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;

public final class FileHelper {
    private FileHelper() { }

    public static void WriteFile(String filename, String data)
    {
        FileWriter out = null;
        try {
            File f = new File("/files/", filename);
            if(!f.exists()) {
                f.createNewFile();
            }
            out = new FileWriter(f, false);
            out.write(data);
        }
        catch(Exception e){
            Log.e("uh oh", "file write broke");
        }
        finally {
            if(out != null) {
                try {
                    out.close();
                }
                catch (IOException e) { }
            }
        }
    }

    public static String ReadFile(String filename){
        FileInputStream in = null;
        String ret = null;
        try {
            File file = new File("/files/", "a.txt");
            if(!file.exists()){
                file.createNewFile();
            }
            in = new FileInputStream(file);
            byte[] data = new byte[(int) file.length()];
            in.read(data);
            ret = new String(data, "UTF-8");
        }
        catch(Exception e){
            Log.e("uh oh", "file read broke");
        }
        finally{
            if(in != null){
                try{
                    in.close();
                }
                catch (IOException e) { }
            }
        }
        return ret;
    }
}
