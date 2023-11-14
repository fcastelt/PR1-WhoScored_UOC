from browser import Browser
from scraper import WhoScoredScrapper

# Realitzem el webscraping, servint-nos de la classe WhoScoredScreapper (que és l'scraper pròpiament) i del navegador
# que aixeca la classse browser.
with Browser() as browser:
    print("Comprovem el user agent: ", browser.user_agent)
    scraper = WhoScoredScrapper(browser)

    # Li passem els paràmetres que volem scrapejar. Cal passar-li un array amb les lligues (vegeu Readme per les
    # lligues disponibles) i una temporada.

    scraper.scrape(["LaLiga", "Premier League", "Serie A", "Ligue 1", "Bundesliga"], '2022/2023')
    scraper._data2csv('dataset.csv')
