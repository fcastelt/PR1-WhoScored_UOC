from bs4 import BeautifulSoup
import re

# Classe que parseja l'html que li passem. De fet, rep com a argument un html
class WhoScoredParser:
    def __init__(self, html):
        self.parser = BeautifulSoup(html, 'html.parser')

    # Funció per extreure elements en format text de l'html. En aquest cas li passem l'etiqueta a buscar i el
    # diccionari amb paràmetres que vulguem trobar (classes, attributs, etc)
    def get_text_element(self, tag, search_params=None, nested_find=None, base_element=None):
        # Contemplem diversos casos. Un d'ells que del fitxer html gran vulguem extreure'n un fragment més petit per fer
        # iteracions en les cerques. En aquest cas, li indicarem aquest element per fer la iteració
        # Busquem l'element controlant per possibles errors i excepcions. En cas que no ho trobi o hi hagi error
        # retornarà un conjunt buit, però no pararà la cerca.
        try:
            if base_element:
                if search_params:

                    # O que vulguem fer scraping d'elements anidats. En aquest cas, indicarem un paràmetre nested_find,
                    # que d'entrada no és obligatori
                    if nested_find:
                        result = base_element.find(tag, **search_params).find(**nested_find).text
                    else:
                        result = base_element.find(tag, **search_params).text
                else:
                    if nested_find:
                        result = base_element.find(tag).find(**nested_find).text
                    else:
                        result = base_element.find(tag).text
            else:
                if search_params:
                    if nested_find:
                        result = self.parser.find(tag, **search_params).find(**nested_find).text
                    else:
                        result = self.parser.find(tag, **search_params).text
                else:
                    if nested_find:
                        result = self.parser.find(tag).find(**nested_find).text
                    else:
                        result = self.parser.find(tag).text
        except:
            result = ""

        return result

    # Funció per obtenir la llista de partits, a través d'un id, i buscant totes les classes indicadores. Retorna una
    # llista.
    def get_matches_url(self) -> list[str]:
        return list(map(lambda x: x.get('href'),
                        self.parser.find(id="tournament-fixture-wrapper").find_all('a', class_='result-1 rc')))


    # Funció que retora un booleà que ens indica si el títol del botó de la setmana és el darrer o no. Ens serveix per
    # evitar bucles ja que quan la funció prengui el valor True, pararà
    def no_previous_week(self):
        week_title = self.parser.find('a', {'class': re.compile('previous button.*')}).get('title')
        title = "No data for previous week"
        return week_title == title

    # Funció per obtenir la info general de l'equip. Fem servir la funció text_element per aconseguir les dades
    def get_team_info(self, html):
        team_name = self.get_text_element('a', {'class_': 'team-name'}, base_element=html)
        coach = self.get_text_element('span', {'class_': 'manager-name'}, base_element=html)
        formation = self.get_text_element('div', {'class_': 'formation'}, base_element=html)
        return team_name, coach, formation

    # Funció per obtenir els stats per equip. En aquest cas, hi ha un paràmetre home_away que, segons li indiquem
    # ens retornarà la dada per l'equip local o visitant. La resta, no canvia i és un get_text_element estàndar.
    def get_stat_by_team(self, tag, html, home_away):
        stat = self.get_text_element(tag, {'data-field': home_away}, base_element=html)
        return stat

    # Funció per obtenir la info general del matx. Retorna una array amb el nom de les variables i una altra amb les
    # dades
    def match_info_and_possession(self):

        # Creem una llista per emmagatzemar la informació
        match_info_and_possession = []

        # Definim la inforamació a obtenir. En el cas de la data, l'obtenim amb regex, perquè era la manera òptima per
        # obtenir totes les dades
        date = self.get_text_element('dd', {'text': re.compile(r'^[A-Za-z]{3}, \d{2}-[A-Za-z]{3}-\d{2}$')})
        referee = self.get_text_element('span', {'class_': 'referee'}, {'class_': 'value'})
        stadium = self.get_text_element('span', {'class_': 'venue'}, {'class_': 'value'})

        # Obtenim l'element amb les dades genèriques de l'equip
        home_team_info = self.parser.find('div', class_='match-centre-header-team', attrs={'data-field': 'home'})
        away_team_info = self.parser.find('div', class_='match-centre-header-team', attrs={'data-field': 'away'})
        possession_info = self.parser.find('li', class_='match-centre-stat has-stats', attrs={'data-for': 'possession'})

        # Usant les funcions abans comentades, extraiem la informació necessària. En aquest cas, get_team_info, ja té
        # els paràmetres que ha de buscar i només necessita que li passem un fragment d'html per parsejar-lo
        home_team, home_coach, home_formation = self.get_team_info(home_team_info)
        away_team, away_coach, away_formation = self.get_team_info(away_team_info)
        home_possession, away_possession = (self.get_stat_by_team('span', possession_info, 'home'),
                                            self.get_stat_by_team('span', possession_info, 'away'))


        # Finalment, ho annexem tot a una llista

        match_info_and_possession.extend((date, referee, stadium,
                                          home_team, home_coach, home_formation,
                                          away_team, away_coach, away_formation,
                                          home_possession, away_possession
                                          ))
        header_match_info_and_possession = [name for name, value in locals().items()
                                            if value in match_info_and_possession]

        return header_match_info_and_possession, match_info_and_possession

    # Funció per obtenir les estadístiques del matx. Retorna una array amb el nom de les variables i una altra amb les
    # dades
    def get_match_stats(self):

        # Llistes on emmagatzarem la informació
        header_match_stats = []
        match_stats = []

        # Diccionari on emmagatzemar els índex de les estadístiques genèriques per després fer servir d'arrel del nom de
        # les estadístiques detallades
        stats_dict = {}

        # Obtenim tots els elements que tinguin el tag i la classe indicada. Per cada cas, seran les estadístiques
        # generals o les estadístiques detallades.
        general_stats = self.parser.find_all('li', class_='filterz-option')
        detailed_stats = self.parser.find_all('div', class_='filterz-filter')

        # Iterem per cada element trobat en les variables general_stats i extraiem la informació necessària. També
        # emmagatzamem l´índex en el didccionari.
        for stat in general_stats:
            home_general_stat = self.get_stat_by_team('span', stat, 'home')
            away_general_stat = self.get_stat_by_team('span', stat, 'away')
            general_stat_name = (self.get_text_element('h4', base_element=stat)).lower().strip().replace(" ", "_")
            general_stat_index = int(stat['data-filter-index'])

            if general_stat_index not in stats_dict:
                stats_dict[general_stat_index] = general_stat_name
            else:
                pass

            match_stats.extend((home_general_stat, away_general_stat))
            header_match_stats.extend(('home_' + general_stat_name, 'away_' + general_stat_name))

        # Iterem per cada element trobat en les variables detailed_stats i extraiem la informació necessària.
        for detailed_stat in detailed_stats:

            # Cerquem l'índex de cada estadística
            detailed_stat_index = detailed_stat.get('data-filter-index')

            #En aquest cas descartem l'índex all perquè no ens interessa (no té informació)
            if detailed_stat_index == 'all':
                continue

            # De l'índex detallat, que te format x_x_x, extraíem el primer digit (fins al símbol "_") que coincideix amb
            # l'índex de l'estadística general
            root_detailed_stat_index = int(re.search(r"(.*?)_", detailed_stat_index).group(1))

            # Extraiem les estadístiques
            home_detailed_stat = self.get_stat_by_team('span', detailed_stat, 'home')
            away_detailed_stat = self.get_stat_by_team('span', detailed_stat, 'away')

            detailed_stat_name = ((self.get_text_element('label', base_element=detailed_stat)).
                                  lower().strip().replace(" ", "_"))

            # Pel nom de l'estadística, li afegim l'arrel de l'estadística general de la qual depèn, la qual cerquem en
            # el diccionari prèviament creat.
            complete_detailed_stat_name = stats_dict[root_detailed_stat_index] + "_" + detailed_stat_name

            match_stats.extend((home_detailed_stat, away_detailed_stat))
            header_match_stats.extend(('home_' + complete_detailed_stat_name, 'away_' + complete_detailed_stat_name))

        return header_match_stats, match_stats
