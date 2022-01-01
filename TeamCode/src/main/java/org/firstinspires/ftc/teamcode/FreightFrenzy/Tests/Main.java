package org.firstinspires.ftc.teamcode.FreightFrenzy.Tests;

import java.io.PrintWriter; // To write to files
import java.util.Scanner; // To take inputs
import java.io.File;  // Import the File class
import java.io.FileWriter;   // Import the FileWriter class
import java.io.IOException;  // Import the IOException class to handle errors

public class Main {
    public static String fileName = "Collatz";
    public static String fileExtension = ".csv";
    public static FileWriter fileWriter;
    public static PrintWriter printWriter;
    public static int num0;
    public static int num1;
    public static int num2;
    public static int num3;

    private static void createFile() {
        try {
            File myObj = new File(fileName + fileExtension);
            int addNum = 1;
            while(myObj.exists() && !myObj.isDirectory()) {
                //noinspection StringConcatenationInLoop
                fileName += "(" + string(addNum) + ")";
                addNum++;
                myObj = new File(fileName + fileExtension);
            }
            if (myObj.createNewFile()) {
                System.out.println("File created: " + myObj.getName());
            }
            else {
                System.out.println("File already exists.");
            }
        }
        catch (IOException e) {
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }

    public static void author(String quote) {
            printWriter.print(quote);
    }

    private static void getInputs() {
        Scanner input0 = new Scanner(System.in);
        System.out.println("Enter number 1: ");
        num0 = input0.nextInt();

        Scanner input1 = new Scanner(System.in);
        System.out.println("Enter number 2: ");
        num1 = input1.nextInt();

        Scanner input2 = new Scanner(System.in);
        System.out.println("Enter number 3: ");
        num2 = input2.nextInt();

        Scanner input3 = new Scanner(System.in);
        System.out.println("Enter number 4: ");
        num3 = input3.nextInt();
    }

    private static boolean isActive(int num) {
        return num != 1;
        /*
        Same as:
        if(num == 1){return true;}
        else{return false;}
         */
    }

    private static String string(int num) {
        return Integer.toString(num);
    }

    private static int collatz(int num) {
        if(isActive(num)) {
            if(num % 2 != 0){
                num *= 3;
                num++;
            }
            else {
                num /= 2;
            }
            return num;
        }
        else {
            return 1;
        }
    }

    public static void main(String[] args) throws IOException {
        createFile();
        fileWriter = new FileWriter(fileName + fileExtension);
        printWriter = new PrintWriter(fileWriter);
        getInputs();
        author(string(num0) + "," + string(num1) + "," + string(num2) + "," + string(num3) + "\n");
        while(isActive(num0) || isActive(num1) || isActive(num2) || isActive(num3)) {
            num0 = collatz(num0);
            num1 = collatz(num1);
            num2 = collatz(num2);
            num3 = collatz(num3);
            author(string(num0) + "," + string(num1) + "," + string(num2) + "," + string(num3) + "\n");
        }
        printWriter.close();
    }
}
