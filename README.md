# Introduzione
Esperienza di laboratorio per il corso **Dinamica e modelli di robot** tenuto dai professori Bortoluzzi Daniele e Moretti Giacomo, anno accademico 2025/2026. Syllabus del corso consultabile al seguente [link](https://unitn.coursecatalogue.cineca.it/corsi/2023/10131/insegnamenti/2025/50182_640667_96084/2020/50182?annoOrdinamento=2020).

# Installazione
1) Aprite un terminale (combinazione di tasti `ctrl`+`alt`+`T`)
2) Cambiate la cartella di lavoro a `Desktop`
    ```sh
    cd ~/Desktop
    ```
3) Create la cartella per l'esperienza
    ```
    mkdir dmr_2026 && cd dmr_2026
    ```
3) Clonate il seguente progetto
    ```
    git clone https://github.com/davidedema/dinamica-e-modelli-di-robot-lab.git
    ```
4) Eseguite il file `install.sh` per la creazione dell'ambiente di lavoro
    ```
    chmod +x install.sh && ./install.sh
    ```
5) Come ultimo output dovrebbe comparire `Connection successful`

# Utilizzo
Per utilizzare il programma:

1) Cambiare la cartella di lavoro con quella del programma
    ```sh
    cd ~/Desktop/dmr_2026/dinamica-e-modelli-di-robot-lab
    ```
2) Eseguire lo script `main.py`. Esistono varie opzioni per lo script
    - `--mount-pen` Aggiungere questo flag se si desidera inserire il marker
    - `--remove-pen` Aggiungere questo flag se si desidera rimuovere il marker
    - `--simulation` Aggiungere questo flag se si desidera simulare il robot invece di controllarlo nella realtà
    - `--points` Flag **obbligatorio**, serve per specificare i punti nel piano XY da raggiungere, vanno inseriti nel formato X1,Y1 X2,Y2 ... I limiti del workspace sono $X\in[0.2, 0.3]$ e $Y\in[-0.1,0.1]$
    > uso --points 0.2,0.05 0.21,-0.03 
    - `--n-times` Flag che specifica quante volte ripetere il movimento verso un punto
    > uso --n-times 4
    - `--serial-port` Flag per specificare la porta seriale (default `/dev/ttyACM0`)

## Esempio
Se vogliamo inserire il marker, raggiungere due punti (0.2,0.05 0.21,-0.03) per due volte l'uno
```
python3 main.py --mount-pen --points 0.2,0.05 0.21,-0.03 --n-times 2
```