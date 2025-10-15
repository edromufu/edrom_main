import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import os
import math
from dynamixel_sdk import *

# -- Configurações do seu Robô --
DEVICE_NAME                 = '/dev/ttyUSB1'
BAUDRATE                    = 1000000
MOTOR_IDS                   = list(range(3, 21)) # IDs de 9 a 20

# -- Protocolo e Endereços de Registradores (Padrão para a maioria dos motores XL, XM, XH) --
PROTOCOL_VERSION            = 2.0
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
LEN_GOAL_POSITION           = 4

# -- Inicialização da SDK --
portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

try:
    portHandler.openPort()
    print("Porta serial aberta com sucesso.")
    portHandler.setBaudRate(BAUDRATE)
    print(f"Baudrate definido para {BAUDRATE}.")
except Exception as e:
    print(f"Erro ao inicializar a porta serial: {e}")
    messagebox.showerror("Erro de Hardware", 
                         "Não foi possível conectar ao U2D2. Verifique a porta e as permissões.")
    exit()

def set_torque_robo(enable: bool):
    status_str = "Ligando" if enable else "Desligando"
    status_val = 1 if enable else 0
    print(f"[ROBÔ] {status_str} torque para todos os motores...")
    
    for motor_id in MOTOR_IDS:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, status_val)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print(f"Falha ao {status_str.lower()} torque do motor {motor_id}")
            return False
    print(f"[ROBÔ] Torque {status_str.lower()} com sucesso.")
    return True

def aplicar_offsets_no_robo(offsets_rad: dict):
    print("[ROBÔ] Aplicando offsets nos motores...")
    
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
    
    for joint_name, offset_rad in offsets_rad.items():
        try:
            motor_id = int(joint_name.split('_')[1])
        except (ValueError, IndexError):
            continue

        if motor_id in MOTOR_IDS:
            # Posição final = 0 (centro) + offset
            final_angle_rad = offset_rad
            
            # Converte radianos para "ticks" (0-4095)
            # A fórmula é: (ângulo_rad * 2048 / pi) + 2048
            pos_ticks = int((final_angle_rad * 2048 / math.pi) + 2048)
            
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos_ticks)),
                                   DXL_HIBYTE(DXL_LOWORD(pos_ticks)),
                                   DXL_LOBYTE(DXL_HIWORD(pos_ticks)),
                                   DXL_HIBYTE(DXL_HIWORD(pos_ticks))]
            
            groupSyncWrite.addParam(motor_id, param_goal_position)

    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Erro ao enviar posições: {packetHandler.getTxRxResult(dxl_comm_result)}")

    groupSyncWrite.clearParam()
    print("[ROBÔ] Offsets aplicados.")


class CalibrationApp:
    def __init__(self, janela):
        self.janela = janela
        self.janela.title("Ferramenta de Calibração de Offsets")
        
        # Dicionários para guardar os dados e os widgets
        self.offsets = {}
        self.sliders = {}
        self.entries = {}
        self.value_labels = {}

        # --- Layout da Interface ---
        main_frame = ttk.Frame(janela)
        main_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        control_frame = ttk.Frame(janela, padding="10")
        control_frame.pack(fill="x")

        # --- Criação dos Componentes para cada motor ---
        for motor_id in MOTOR_IDS:
            joint_name = f"motor_{motor_id}"
            self.offsets[joint_name] = 0.0

            frame = ttk.Frame(scrollable_frame)
            frame.pack(fill="x", pady=2, padx=5)
            
            ttk.Label(frame, text=f"ID {motor_id}:", width=8).pack(side="left")

            slider = ttk.Scale(frame, from_=-1.57, to=1.57, orient="horizontal",
                               command=lambda val, jn=joint_name: self.on_slider_move(val, jn))
            slider.pack(side="left", fill="x", expand=True, padx=5)
            self.sliders[joint_name] = slider

            value_label = ttk.Label(frame, text="0.0000", width=8)
            value_label.pack(side="left")
            self.value_labels[joint_name] = value_label

            entry = ttk.Entry(frame, width=8)
            entry.pack(side="left", padx=5)
            entry.bind("<Return>", lambda e, jn=joint_name: self.on_entry_confirm(e, jn))
            self.entries[joint_name] = entry

            zero_button = ttk.Button(frame, text="Zerar", width=6,
                                     command=lambda jn=joint_name: self.zero_offset(jn))
            zero_button.pack(side="left")

        # --- Botões de Controle ---
        self.torque_button = ttk.Button(control_frame, text="Ligar Torque", command=self.alternar_torque)
        self.torque_button.pack(side="left", expand=True, padx=5)
        self.torque_ligado = False

        ttk.Button(control_frame, text="Aplicar Offsets", command=self.aplicar_e_ver).pack(side="left", expand=True, padx=5)
        ttk.Button(control_frame, text="Salvar Offsets", command=self.salvar_offsets).pack(side="left", expand=True, padx=5)

        self.carregar_offsets()

    def on_slider_move(self, value, joint_name):
        offset_val = float(value)
        self.offsets[joint_name] = offset_val
        # Atualiza apenas os outros widgets para evitar recursão
        self.value_labels[joint_name].config(text=f"{offset_val:.4f}")
        self.entries[joint_name].delete(0, tk.END)
        self.entries[joint_name].insert(0, f"{offset_val:.4f}")

    def on_entry_confirm(self, event, joint_name):
        try:
            new_value = float(self.entries[joint_name].get())
            new_value = max(-1.57, min(1.57, new_value)) # Garante que o valor está no range
            self.offsets[joint_name] = new_value
            # Atualiza apenas os outros widgets
            self.value_labels[joint_name].config(text=f"{new_value:.4f}")
            self.sliders[joint_name].set(new_value)
        except ValueError:
            messagebox.showerror("Erro de Valor", "Por favor, digite um número válido.")
            # Reseta a caixa de texto para o valor antigo
            old_value = self.offsets.get(joint_name, 0.0)
            self.entries[joint_name].delete(0, tk.END)
            self.entries[joint_name].insert(0, f"{old_value:.4f}")

    def zero_offset(self, joint_name):
        self.offsets[joint_name] = 0.0
        # Atualiza todos os widgets, pois foi um clique de botão
        self.sliders[joint_name].set(0.0)
        self.value_labels[joint_name].config(text="0.0000")
        self.entries[joint_name].delete(0, tk.END)
        self.entries[joint_name].insert(0, "0.0000")
    
    def carregar_offsets(self):
        if os.path.exists("offsets.json"):
            try:
                with open("offsets.json", 'r') as f:
                    offsets_carregados = json.load(f)
                
                for joint_name, offset_val in offsets_carregados.items():
                    if joint_name in self.offsets:
                        self.offsets[joint_name] = offset_val
                        # Atualiza toda a UI ao carregar
                        self.sliders[joint_name].set(offset_val)
                        self.value_labels[joint_name].config(text=f"{offset_val:.4f}")
                        self.entries[joint_name].delete(0, tk.END)
                        self.entries[joint_name].insert(0, f"{offset_val:.4f}")
                print("Offsets carregados de 'offsets.json'.")
            except Exception as e:
                print(f"Não foi possível carregar 'offsets.json': {e}")
        else:
            print("Nenhum arquivo 'offsets.json' encontrado. Começando com zeros.")

    def salvar_offsets(self):
        try:
            with open("offsets.json", 'w') as f:
                json.dump(self.offsets, f, indent=4)
            messagebox.showinfo("Sucesso", "Arquivo 'offsets.json' salvo com sucesso!")
        except Exception as e:
            messagebox.showerror("Erro ao Salvar", f"Não foi possível salvar o arquivo.\nErro: {e}")

    def alternar_torque(self):
        if not self.torque_ligado:
            if set_torque_robo(enable=True):
                self.torque_ligado = True
                self.torque_button.config(text="Desligar Torque")
        else:
            if set_torque_robo(enable=False):
                self.torque_ligado = False
                self.torque_button.config(text="Ligar Torque")

    def aplicar_e_ver(self):
        if not self.torque_ligado:
            messagebox.showwarning("Aviso", "Ligue o torque antes de aplicar os offsets.")
            return
        aplicar_offsets_no_robo(self.offsets)

    def on_closing(self):
        print("Fechando a porta serial.")
        if self.torque_ligado:
            set_torque_robo(enable=False)
        portHandler.closePort()
        self.janela.destroy()

if __name__ == "__main__":
    janela_principal = tk.Tk()
    app = CalibrationApp(janela_principal)
    janela_principal.protocol("WM_DELETE_WINDOW", app.on_closing)
    janela_principal.mainloop()