#!/usr/bin/env python3
"""
Script fusionné DÉMON MOODE-FIRST + AUTO-PLAY RADIOS (HYBRIDE 45s MAX)
Phase 1 MPD rapide (5s) + Phase 2 moodeutl précis (40s)
User: ducretet + FIX GPIO cleanup
"""

import RPi.GPIO as GPIO
import time
import subprocess
import signal
import sys
import threading
import os
import logging

# Logging user ducretet
logging.basicConfig(filename='/home/ducretet/ducretet_control.log', level=logging.INFO,
                    format='%(asctime)s %(levelname)s %(message)s')

# ==================== GLOBAL CONTROL ====================
shutdown_flag = threading.Event()
threads = []
gpio_initialized = False

# Broches BCM
EC2_CLK, EC2_DT, EC2_SW = 5, 6, 12
LED1, LED2 = 17, 27
SPC_SHUTDOWN_IN, SPC_BOOT_OK, SPC_SOFT_SHUTDOWN = 13, 22, 26
RELAY = 4

def setup_gpio():
    """GPIO BCM unifie - Thread-safe"""
    global gpio_initialized
    if gpio_initialized: return
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # INPUTS
    GPIO.setup([EC2_CLK, EC2_DT, EC2_SW, SPC_SHUTDOWN_IN], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # OUTPUTS
    GPIO.setup([LED1, LED2, SPC_BOOT_OK, SPC_SOFT_SHUTDOWN, RELAY], GPIO.OUT)
    
    # ÉTAT INITIAL CRITIQUE
    GPIO.output(SPC_BOOT_OK, GPIO.HIGH)
    GPIO.output(SPC_SOFT_SHUTDOWN, GPIO.LOW)
    GPIO.output(RELAY, GPIO.HIGH)
    
    gpio_initialized = True
    logging.info("SPC REG: BootOK + Relais actives")

def safe_gpio_cleanup():
    """Cleanup GPIO SANS erreur"""
    global gpio_initialized
    if not gpio_initialized: return
    
    try:
        GPIO.output(RELAY, GPIO.LOW)
        GPIO.output(SPC_BOOT_OK, GPIO.LOW)
        GPIO.cleanup()
        logging.info("GPIO cleanup OK")
    except Exception as e:
        logging.error(f"GPIO cleanup erreur ignorée: {e}")
    finally:
        gpio_initialized = False

def cleanup():
    """Arrêt propre + cleanup robuste"""
    logging.info("Arrêt propre des threads...")
    shutdown_flag.set()
    
    # Attendre threads (timeout 2s)
    for thread in threads[:]:
        try:
            thread.join(timeout=2)
        except RuntimeError:
            pass
    
    safe_gpio_cleanup()
    logging.info("Script arrêté proprement")
    sys.exit(0)

def is_bluetooth_active():
    if shutdown_flag.is_set(): return False
    try:
        output = subprocess.check_output(['bluetoothctl', 'info'], text=True, timeout=3).strip()
        return 'Connected: yes' in output
    except:
        return False

def play_radios_playlist():
    """Charge, volume 25% et joue la playlist Radios"""
    if shutdown_flag.is_set(): return False
    try:
        subprocess.run(['mpc', 'clear'], capture_output=True)
        subprocess.run(['mpc', 'load', 'Radios'], capture_output=True)
        subprocess.run(['mpc', 'volume', '25'], capture_output=True)
        subprocess.run(['mpc', 'play'], capture_output=True)
        logging.info("Playlist Radios chargée - Volume 25% - Lecture")
        return True
    except Exception as e:
        logging.error(f"Erreur play_radios_playlist: {e}")
        return False

def mpc_rotary_control():
    """Controle MPC - DESACTIVE en Bluetooth"""
    last_clk_state = GPIO.input(EC2_CLK)
    last_sw_state = GPIO.input(EC2_SW)
    sw_debounce_time = 0
    
    while not shutdown_flag.is_set():
        try:
            if is_bluetooth_active():
                last_clk_state = GPIO.input(EC2_CLK)
                last_sw_state = GPIO.input(EC2_SW)
                time.sleep(0.1)
                continue
            
            current_clk = GPIO.input(EC2_CLK)
            current_dt = GPIO.input(EC2_DT)
            current_sw = GPIO.input(EC2_SW)
            
            if current_clk != last_clk_state and current_clk == 0:
                if current_dt == 1:
                    subprocess.run(['mpc', 'next'], capture_output=True)
                else:
                    subprocess.run(['mpc', 'prev'], capture_output=True)
                last_clk_state = current_clk
            
            if current_sw == 0 and last_sw_state == 1 and (time.time() - sw_debounce_time) > 0.3:
                subprocess.run(['mpc', 'toggle'], capture_output=True)
                sw_debounce_time = time.time()
            
            last_sw_state = current_sw
            last_clk_state = current_clk
        except Exception:
            pass
        time.sleep(0.005)

def bluetooth_led_monitor():
    """Surveillance Bluetooth + LEDs"""
    while not shutdown_flag.is_set():
        try:
            connected = is_bluetooth_active()
            if connected:
                GPIO.output(LED1, GPIO.LOW)
                GPIO.output(LED2, GPIO.HIGH)
            else:
                GPIO.output(LED1, GPIO.HIGH)
                GPIO.output(LED2, GPIO.LOW)
        except Exception:
            pass
        time.sleep(2)

def wait_moode_ready():
    """Attente Moode HYBRIDE OPTIMISÉE (45s MAX)"""
    logging.info("SPC REG actif - Attente Moode HYBRIDE (45s max)...")
    
    # === PHASE 1: MPD rapide (5s max) ===
    logging.info("Phase 1: Check MPD rapide (5s max)...")
    for i in range(10):
        if shutdown_flag.is_set(): return False
        try:
            result = subprocess.run(['mpc', 'status'], capture_output=True, timeout=0.5, text=True)
            if result.returncode == 0:  # MPD répond
                logging.info("Phase 1 OK: MPD répond → Phase 2")
                break
        except:
            pass
        time.sleep(0.5)
    
    # === PHASE 2: moodeutl précis (40s max) ===
    logging.info("Phase 2: Check moodeutl précis (40s max)...")
    retry_count = 0
    while retry_count < 40 and not shutdown_flag.is_set():
        try:
            result = subprocess.check_output([
                '/usr/local/bin/moodeutl', '-q',
                "select value from cfg_system where param='wrkready'"
            ], timeout=1)  # 1s timeout
            if result.strip() == b'1':
                logging.info("Moode fully ready → AUTO-PLAY Radios")
                play_radios_playlist()
                return True
        except:
            pass
        retry_count += 1
        time.sleep(1)  # 1s sleep
    
    logging.warning("Moode timeout après 45s - Services essentiels actifs")
    return False

def spc_shutdown_monitor():
    """Shutdown CRITIQUE - TOUJOURS actif"""
    while True:
        try:
            if GPIO.input(SPC_SHUTDOWN_IN) == 1:
                logging.info("Shutdown detecte → Relais OFF")
                GPIO.output(RELAY, GPIO.LOW)
                GPIO.output(SPC_SOFT_SHUTDOWN, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(SPC_SOFT_SHUTDOWN, GPIO.LOW)
                os.system('sudo shutdown -h -P now')
                break
        except:
            break
        if shutdown_flag.is_set():
            break
        time.sleep(0.1)

# ==================== MAIN MOODE-FIRST ====================
def main():
    signal.signal(signal.SIGINT, lambda s,f: cleanup())
    signal.signal(signal.SIGTERM, cleanup)
    
    try:
        setup_gpio()
        
        # Shutdown thread (CRITIQUE)
        shutdown_thread = threading.Thread(target=spc_shutdown_monitor)
        threads.append(shutdown_thread)
        shutdown_thread.start()
        logging.info("Shutdown monitor actif (BCM 13)")
        
        # ATTENTE MOODE HYBRIDE RAPIDE (45s MAX)
        moode_ready = wait_moode_ready()
        
        if moode_ready and not shutdown_flag.is_set():
            mpc_thread = threading.Thread(target=mpc_rotary_control, daemon=True)
            bt_thread = threading.Thread(target=bluetooth_led_monitor, daemon=True)
            threads.extend([mpc_thread, bt_thread])
            mpc_thread.start()
            bt_thread.start()
            logging.info("TOUS SERVICES ACTIFS - CODEUR OFF EN BT")
        
        while not shutdown_flag.is_set():
            time.sleep(1)
            
    except KeyboardInterrupt:
        logging.info("Ctrl+C détecté")
    except Exception as e:
        logging.error(f"Erreur critique: {e}")
    finally:
        cleanup()

if __name__ == "__main__":
    main()
