from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.service import Service
from webdriver_manager.chrome import ChromeDriverManager
import time

# Creem una value class per tenir units els elements By i element, que sempre van junts
class Target:
    def __init__(self, by: By, element: str):
        self.by = by
        self.element = element


# Creem dues classes per fer els clics. Una d'elles controla quan els clics puguin fallar (cas de popups que a vegades
# apareixen i a vegades no)
class Click:
    def __init__(self, target: Target, wait_time: float = 3):
        self.target = target
        self.wait_time = wait_time

    def could_fail(self):
        return False


class FailableClick(Click):
    def __init__(self, target: Target, wait_time: float = 3):
        super().__init__(target, wait_time)

    def could_fail(self):
        return True

# Creem una classe Browser que aixeca un navegador amb Chorme genèric
class Browser:

    # Definim algunes opcions per evitar timeouts. Extret d'stackoverflow
    # https://stackoverflow.com/questions/48450594/selenium-timed-out-receiving-message-from-renderer
    def __init__(self):
        self.options = Options()
        self.options.add_argument("--disable-renderer-backgrounding")
        self.options.add_argument("--disable-backgrounding-occluded-windows")
        self.options.add_argument("--start-maximized")
        self.options.add_argument("--enable-automation")
        self.options.add_argument("--disable-dev-shm-usage")
        self.options.add_argument("--disable-browser-side-navigation")
        self.options.add_argument("--disable-gpu")
        self._driver = webdriver.Chrome(service=Service(ChromeDriverManager().install()), options=self.options)
        self.user_agent = self._driver.execute_script("return navigator.userAgent")
        self._driver.set_window_position(-2000, 0)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, exception_traceback):
        self._driver.quit()

    def find_element(self, by: By, element: str):
        return self._driver.find_element(by, element)

    def get(self, url) -> None:
        self._driver.get(url)

    def click(self, click: Click) -> None:
        target = click.target
        if click.could_fail():
            try:
                self._driver.find_element(target.by, target.element).click()
            except Exception:
                pass
        else:
            self._driver.find_element(target.by, target.element).click()

        time.sleep(click.wait_time)

    #Funció per calcular el temps de càrrega del servidor. Serà el paràmetre a fer servir pels wait_times entre requests
    def browser_load_time(self) ->int:
        navigationStart = self._driver.execute_script("return window.performance.timing.navigationStart")
        responseStart = self._driver.execute_script("return window.performance.timing.responseStart")
        load_time = (responseStart - navigationStart)/1000
        return load_time

    # Funció que retorna l'html per passar-li al parser
    def html(self) -> str:
        return self._driver.page_source
