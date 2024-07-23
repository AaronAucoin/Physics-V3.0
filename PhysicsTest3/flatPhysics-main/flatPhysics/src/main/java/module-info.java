module com.example.flatphysics {
    requires javafx.controls;
    requires javafx.fxml;


    opens com.example.flatphysics to javafx.fxml;
    exports com.example.flatphysics;
}