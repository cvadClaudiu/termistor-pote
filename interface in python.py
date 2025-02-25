import tkinter as tk
from tkinter import ttk
import serial
import threading
import time

class UARTDisplay:
    def __init__(self, root):
        self.root = root
        self.root.title("Interfata UART")
       
        try:
            self.serial_port = serial.Serial(
                port='COM7',  
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=1
            )
        except serial.SerialException:
            print("Nu s-a putut deschide portul serial!")
            return

       
        self.create_widgets()
        self.running = True
        self.thread = threading.Thread(target=self.read_uart)
        self.thread.daemon = True
        self.thread.start()

    def create_widgets(self):
       
        frame = ttk.LabelFrame(self.root, text="Valori: ", padding="10")
        frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        ttk.Label(frame, text="Temperatura: ").grid(row=0, column=0, padx=5, pady=5)
        self.temp_var = tk.StringVar(value="-- C")
        ttk.Label(frame, textvariable=self.temp_var, font=('Arial', 14, 'bold')).grid(row=0, column=1, padx=5, pady=5)

        
        ttk.Label(frame, text="Factor de umplere:").grid(row=1, column=0, padx=5, pady=5)
        self.duty_var = tk.StringVar(value="--%")
        ttk.Label(frame, textvariable=self.duty_var, font=('Arial', 14, 'bold')).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(frame, text="Mod operare:").grid(row=2, column=0, padx=5, pady=5)
        self.mode_var = tk.StringVar(value="--")
        ttk.Label(frame, textvariable=self.mode_var, font=('Arial', 14, 'bold')).grid(row=2, column=1, padx=5, pady=5)

    def read_uart(self):
        while self.running:
            if self.serial_port.in_waiting:
                try:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    
                
                    if "Celsius" in line:
                        temp = line.split()[0]
                        self.temp_var.set(f"{temp} grade Celsius")
                    elif "umplere" in line:
                        duty = line.split()[0]
                        self.duty_var.set(f"{duty}")
                    elif "mod_AUTOMAT" in line:
                        self.mode_var.set("mod AUTOMAT")
                    elif "mod_MANUAL" in line:
                        self.mode_var.set("mod MANUAL")
                except Exception as e:
                    print(f"Eroare la citirea UART: {e}")
            
            time.sleep(0.1)

    def cleanup(self):
        self.running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

def main():
    root = tk.Tk()
    app = UARTDisplay(root)
    
    def on_closing():
        app.cleanup()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()

if __name__ == "__main__":
    main()