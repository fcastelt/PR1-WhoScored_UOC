from selenium.webdriver.common.by import By
from selenium.webdriver.support.select import Select
import time
from source.browser import FailableClick
from source.browser import Click
from source.browser import Target
from source.whoscored_parser import WhoScoredParser


# Creem la classe que actuarà pròpiament d'scrapper. Farà els clics i extreurà la informació (que prèviament sol·licitarà
# al parser contingut a whoscored_parser.py
class WhoScoredScrapper:

    # Li indiquem la web a scrappejar
    _DOMAIN = "https://www.whoscored.com"

    def __init__(self, browser):
        self.browser = browser
        self.data = []

    # Funció per executar l'scraping, agafant funcions de la classe browser i de la classe whoscoredparser.
    def scrape(self, leagues, season):

        # Aixequem un navegador que vagi a la web indicada a _DOMAIN.
        self.browser.get(self._DOMAIN)
        self._close_popups()
        matches_to_scrape = []

        # Per cada lliga passada com a paràmetre de la funció, li indiquem que busqui les urls (amb la funció _navigate_weeks
        # i les guardi en una llista
        for league in leagues:
            league_matches = self._navigate_weeks(league, season)
            matches_to_scrape.extend(league_matches)

        print("S'han trobat {} partits per scrappejar, de {}"
              .format(len(set(matches_to_scrape)), self._DOMAIN))
        print("Depenent del nombre de partits i lligues, l'acció pot tardar hores. L'scraping dura uns 15 segons "
              "per partit.")

        # Per cada url, extraiem les dades necessàries amb la funció _get_stats. En cas que les dades estiguin buides,
        # li diem que també inclogui el header. Sinó, no.
        for match in set(matches_to_scrape):
            if len(self.data) == 0:
                self.data.append(self._get_stats(match)[0])
                self.data.append(self._get_stats(match)[1])
            else:
                self.data.append(self._get_stats(match)[1])
            print("Queden {} partits per scrappejar".format(len(set(matches_to_scrape)) - (len(self.data) - 1)))

    # Funció per tancar els popups que puguin aparèixer
    def _close_popups(self) -> None:
        close_cookies = FailableClick(Target(By.CLASS_NAME, "css-1wc0q5e"), wait_time=0.5)
        close_popup = FailableClick(Target(By.CLASS_NAME, "webpush-swal2-close"), wait_time=0.5)
        self.browser.click(close_cookies)
        self.browser.click(close_popup)

    # Funció per obtenir les diferents urls per setmana, a partir de la funció get_matches_url del parser.
    def _get_match_links_week(self, parser) -> list:
        urls_week = list(map(lambda x: self._DOMAIN + x, parser.get_matches_url()))
        return urls_week

    # Funció per navegar per les setmanes, extreure els links i evitar spidertraps
    def _navigate_weeks(self, league, season) -> list:

        # Primer cliquem sobre la lliga
        self.browser.click(Click(Target(By.LINK_TEXT, league)))

        # Seleccionem la temporada del desplegable
        selected_season = Select(self.browser.find_element(By.ID, "seasons"))
        selected_season.select_by_visible_text(season)

        # En cas que hi hagi fase de playoff o d'ascens i descens, fem que seleccioni la temporada regular. Acostuma a
        # ser el mateix que el de la lliga. En cas que no sigui així li diem que segueixi sense fer-ne cas.
        try:
            selected_stage = Select(self.browser.find_element(By.ID, "stages"))
            selected_stage.select_by_visible_text(league)
        except:
            pass

        previous_week = FailableClick(Target(By.XPATH, '//*[@id="date-controller"]/a[1]'), wait_time=0.5)

        while True:
            html = self.browser.html()
            parser = WhoScoredParser(html)
            url_list = []

            #Amb aquesta condició, evitem spidertraps. Li diem que vagi clicant fins que es trobi amb què ja no hi hagi
            # més dades. Així limitem la profunditat. Sinó, aniria clicant el botó sempre, entrant en un bucle infinit.
            while not parser.no_previous_week():
                try:
                    match_links = self._get_match_links_week(parser)
                    url_list.extend(match_links)
                except Exception:
                    continue

                self.browser.click(previous_week)
                html = self.browser.html()
                parser = WhoScoredParser(html)

            match_links = self._get_match_links_week(parser)
            url_list.extend(match_links)
            break
        return url_list

    # Aquesta funció emmagatzem les dades que es recullin de cada partit
    def _get_stats(self, url):
        header = []
        stats = []

        # Accedim a la url del partit i fem un wait time equivalent al doble del temps de càrrega de les dades al servidor.
        self.browser.get(url)
        time.sleep(2*self.browser.browser_load_time())
        html = self.browser.html()

        # Passem l'html recollit pel parser i extraiem les dades generals essent [0] el header i [1] les dades.
        parser = WhoScoredParser(html)
        header.extend(parser.match_info_and_possession()[0])
        stats.extend(parser.match_info_and_possession()[1])

        # Fem click al chalkboard i esperem lleugerament a que es carreguin les dades. Aquí considerem que no cal esperar
        # tant, perquè l'estructura de dades ja s'ha carregat prèviament, només cal omplir amb els valors, i perquè després
        # ja deixarem descansar el servidor quan entrem a un altre partit.
        self.browser.click(Click(Target(By.LINK_TEXT, 'Chalkboard'), wait_time=0.5))

        html = self.browser.html()
        parser = WhoScoredParser(html)

        # Finalment extraiem les dades de l'html parsejat essent [0] el header i [1] les dades.
        header.extend(parser.get_match_stats()[0])
        stats.extend(parser.get_match_stats()[1])

        return header, stats

    # Funció per crear el dataset en format csv
    def _data2csv(self, filename):

        # Crea el fitxer si no existeix, i si existeix el sobreescriu
        with open("../dataset/" + filename, 'w+') as csv_file:
            # Anem afegint cada array separada per ";"
            for item in self.data:
                row = ";".join(map(str, item))
                csv_file.write(row + '\n')
