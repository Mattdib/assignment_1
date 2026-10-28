# 3D LiDAR Obstacle Detection & Euclidean Clustering

Progetto in C++ basato su **PCL (Point Cloud Library)** per il rilevamento, la segmentazione e la classificazione degli ostacoli stradali a partire da stream sequenziali di nuvole di punti LiDAR (`.pcd`).

Il sistema include una pipeline completa di elaborazione 3D che comprende filtraggio, rimozione del manto stradale tramite RANSAC, clustering euclideo (sia nativo PCL che tramite implementazione personalizzata di KD-Tree) e classificazione del livello di rischio basata sulla corsia di marcia dell'ego-veicolo.

---

## Pipeline di Elaborazione

La funzione principale `ProcessAndRenderPointCloud` esegue una pipeline di elaborazione sequenziale su ciascun frame LiDAR:

```text
Input Cloud (.pcd) 
    │
    ▼
1. Voxel Grid Downsampling (0.15m)
    │
    ▼
2. ROI CropBox Filtering
    │
    ▼
3. RANSAC Ground Plane Removal
    │
    ▼
4. KD-Tree & Euclidean Clustering (Custom / PCL)
    │
    ▼
5. Bounding Box & Distance Calculation
    │
    ▼
6. Collision Warning & 3D Visualization
```

### 1. Voxel Grid Filtering (Downsampling)
* Riduce la densità della nuvola di punti mantenendo la struttura geometrica della scena per alleggerire il carico computazionale.
* Utilizza una griglia cubica con voxel di dimensione pari a $0.15\text{ m} \times 0.15\text{ m} \times 0.15\text{ m}$ (`vg.setLeafSize(0.15f, 0.15f, 0.15f)`).

### 2. Region of Interest (ROI) Cropping
* Isola lo spazio operativo rilevante per la guida autonoma tramite `pcl::CropBox`, scartando punti troppo distanti o riflessioni non pertinenti.
* **Limiti ROI impostati**:
  * $X \in [-20.0, 30.0]\text{ m}$ (estensione longitudinale lungo la traiettoria)
  * $Y \in [-6.0, 7.0]\text{ m}$ (estensione laterale per coprire corsie adiacenti)
  * $Z \in [-2.0, 5.0]\text{ m}$ (estensione verticale rispetto al sensore)

### 3. Segmentazione e Rimozione del Manto Stradale (RANSAC)
* Modella il piano stradale come componente dominante principale tramite `pcl::SACSegmentation` con modello `SACMODEL_PLANE`.
* Parametri impostati: $100$ iterazioni massime e soglia di distanza punto-piano pari a $0.15\text{ m}$.
* Gli inlier del piano vengono estratti e visualizzati in verde (`cloud_plane`), mentre la nuvola priva del piano (`cloud_filtered`) prosegue verso la fase di clustering.

### 4. Euclidean Clustering
Il progetto supporta due modalità di clustering, selezionabili tramite la direttiva `#define USE_PCL_LIBRARY`:
* **PCL `EuclideanClusterExtraction`**: Algoritmo nativo di libreria ottimizzato su KD-Tree PCL.
* **Custom KD-Tree + Recursive Proximity**: Implementazione tridimensionale proprietaria (`my_pcl::KdTree`) combinata con la funzione ricorsiva `proximity()` per la ricerca dei vicini e l'aggregazione dei punti tramite `std::unordered_set<int>`.
* **Parametri di Clustering**:
  * Tolleranza di vicinanza (`clusterTolerance`): $0.35\text{ m}$ (garantisce la coesione dei punti dello stesso veicolo evitando la fusione di ostacoli distinti).
  * Dimensione minima cluster (`setMinClusterSize`): $15$ punti (intercetta pedoni o veicoli lontani).
  * Dimensione massima cluster (`setMaxClusterSize`): $1000$ punti (evita cluster anomali derivanti da guardrail o pareti continue).

### 5. Bounding Box e Logica di Allerta Collisione
Per ciascun cluster identificato:
1. Viene calcolato il volume minimo/massimo tramite `pcl::getMinMax3D` e generato un bounding box 3D orientato lungo gli assi.
2. Viene stimata la distanza euclidea radiale dal sensore verso il punto di ingombro posteriore dell'ostacolo:
   $$d = \sqrt{x_{\min}^2 + y_{\text{center}}^2}$$
3. Viene renderizzata un'etichetta con la distanza misurata a $+0.4\text{ m}$ sopra il box dell'ostacolo.
4. Viene classificato il livello di allerta considerando una corsia di marcia con semi-ampiezza di $1.5\text{ m}$ ($Y \in [-1.5, 1.5]\text{ m}$):
   * **Critico (Box Rosso)**: Ostacolo nella propria corsia a distanza longitudinale $0 < X \le 5.0\text{ m}$.
   * **Attenzione (Box Giallo)**: Ostacolo nella propria corsia a distanza longitudinale $5.0 < X \le 10.0\text{ m}$.
   * **Sicuro / Altra Corsia (Box Ciano)**: Veicoli oltre i $10.0\text{ m}$, in corsie adiacenti o alle spalle dell'ego-veicolo.

---

## Struttura del Repository

```text
├── CMakeLists.txt
├── include/
│   ├── Renderer.hpp          # Gestione del visualizzatore grafico PCL e camera
│   └── tree_utilities.hpp    # Struttura ad albero my_pcl::KdTree
├── src/
│   └── cluster_extraction.cpp # Pipeline di elaborazione, RANSAC e clustering
└── dataset/                  # Sequenza di file Point Cloud (.pcd)
```

---

## Panoramica dei Moduli (`cluster_extraction.cpp`)

* **`setupKdtree(...)`**: Popola l'albero `my_pcl::KdTree` con i punti 3D della nuvola filtrata.
* **`proximity(...)`**: Funzione ricorsiva che esplora i vicini entro la tolleranza euclidea e costruisce il cluster verificando il vincolo di dimensione massima.
* **`euclideanCluster(...)`**: Coordina l'iterazione globale dei punti, gestendo l'insieme dei nodi già visitati e applicando i vincoli di dimensione minima e massima.
* **`ProcessAndRenderPointCloud(...)`**: Esegue downsampling, ritaglio ROI, estrazione RANSAC, clustering e rendering condizionale dei bounding box.
* **`main(...)`**: Inizializza il visualizzatore, carica in ciclo continuo i frame `.pcd` ordinati cronologicamente da disco e gestisce il refresh dello stream a circa $25\text{ FPS}$.

---

## Prerequisiti e Dipendenze

* **Compilatore C++**: compatibile con standard C++14 o successivo
* **CMake**: versione $\ge 3.10$
* **Point Cloud Library (PCL)**: versione $\ge 1.8$
* **Eigen3**
* **Boost**: moduli `system` e `filesystem`

### Installazione su distribuzioni Linux (Ubuntu / Debian)

```bash
sudo apt-get update
sudo apt-get install -y libpcl-dev libboost-all-dev cmake build-essential
```

---

## Compilazione ed Esecuzione

1. **Configurazione del percorso del dataset**:
   All'interno della funzione `main()` in `cluster_extraction.cpp`, verificare che il percorso del dataset punti alla cartella locale contenente i file `.pcd`:
   ```cpp
   std::vector<boost::filesystem::path> stream(boost::filesystem::directory_iterator{"/path/to/your/dataset"},
                                               boost::filesystem::directory_iterator{});
   ```

2. **Scelta dell'algoritmo di Clustering**:
   * Per usare il KD-Tree custom, lasciare commentata la direttiva `#define USE_PCL_LIBRARY`.
   * Per usare l'implementazione PCL standard, decommentare la direttiva `#define USE_PCL_LIBRARY` all'inizio del file.

3. **Compilazione del progetto**:
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

4. **Esecuzione**:
   ```bash
   ./cluster_extraction
   ```