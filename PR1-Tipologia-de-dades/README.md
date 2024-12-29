# Web Scrapping -- WhoScored.me

Aquest repositori conté codi per a realitzar data scraping a la pàgina web www.whoscored.me mitjançant scripts de Python. Per defecte, el codi genera un fitxer csv amb tots els partits de les lligues Big 5 de la temporada 2022/23 amb estadístiques detallades de cada jornada. 

## Passos per a executar el projecte

Per començar amb el projecte, segueix els següents passos.

1. Clona el repositori a la teva màquina local:

```bash
git clone https://github.com/trepi93/Tipologia-PRA-1.git
```

2. Navega fins al directori del projecte:

```bash
cd Tipologia-PRA-1
```

3. Instala les llibreries necessàries. Es recomana utilitzar un entorn virtual abans de fer-ho:

```bash
pip install -r requirements.txt
```

4. Executa l'scrapping:

   - L'script principal per realitzar l'scrapping i per tant, a executar és "main.py". 
   - Podeu modificar els paràmetres d'entrada de la funció scraper.scrape() segons l'any a descarregar i les lligues d'interès. En cas de les lligues, cal que mantingueu el nom tal i com es troben en l'script i en el cas dels anys, el format és 'YYYY/YYYY'.
   - Per exemple, per a descarregar la temporada 21-22 dela lliga espanyola caldria fer la següent modificació al fitxer main.py:
    scraper.scrape(["La Liga"], '2021/2022')
   - Per executar la descàrrega de dades, executeu l'ordre següent dins de la carpeta source:

   ```bash
   python main.py
   ```

   El csv es generarà a la carpeta dataset.

## Lligues scrapejables

Les lligues descargables es citen a continuació:
-La Liga
-Premier League
-Ligue 1
-Serie A
-Bundesliga
-Liga Portugal
-Super Lig (Turquia)
-Eredivise
-Championship
-Premiership
-2. Bundesliga
-League One
-League Two

Per descarregar altres lligues caldria modificar algun paràmetre per la cerca de temporada o de la fase del campionat que es vol scrapejar.

## Estructura del projecte

El repositori té la següent estructura:

  - `source`: Aquest directori conté el codi font per la descàrrega.
  - `main.py`: conté l’script que acaba executant tot el webscraping
  - `browser.py´: conté el codi necessari per aixecar un browser amb la llibreria Selenium
  - `whoscored_parser.py´: conté el codi necessari per parsejar la web whoscored.com, és a dir per extreure aquells elements que volem.
  - `scraper.py´: conté el codi que fa possible scrapejar la web, a partir dels dos fitxers anteriors. És a dir, conté el codi que navega per la web (a partir de les classes de browser.py) i que extreu les dades (amb funcions que criden les dades al fitxer whoscored_parser.py).
- `dataset`: Aquest directori contindrà el csv generat.
- `requirements.txt`: El fitxer de text amb les llibreries de Python necessàries per aquest projecte.
- `README.md`: L'arxiu actual, que proporciona un resum del projecte.

## Llicència

Aquest conjunt de dades està destinat únicament a fins educatius. Llegir el fitxer de llicència per a més informació.





