# Makefile para Instalação do Webots [EDROM-2025]
# Made by Pedro H. Peres in case of doubt

# --- Variáveis de Configuração ---
WEBOTS_VERSION ?= R2025a
INSTALL_DIR ?= /usr/local/webots
DOWNLOAD_URL = https://github.com/cyberbotics/webots/releases/download/$(WEBOTS_VERSION)/webots-$(WEBOTS_VERSION)-x86-64.tar.bz2

# Checksum SHA256 do arquivo TAR.BZ2 (HEXADECIMAL). ALTAMENTE RECOMENDADO!
# VERIFIQUE SEMPRE O HASH OFICIAL na página de releases do Webots para a versão exata.
DOWNLOAD_CHECKSUM ?= c5127fb4206c57a5ae5523f1b7f3da8b670bc8926d9ae08595e139f226f38c38

# --- Variáveis Internas ---
DOWNLOAD_FILE = /tmp/webots-$(WEBOTS_VERSION).tar.bz2
PROFILE_SCRIPT = /etc/profile.d/webots.sh
CHECKSUM_FILE = /tmp/webots-$(WEBOTS_VERSION).sha256

# --- Targets ---

.PHONY: all install dependencies download extract configure clean help

all: help

help:
	@echo "Uso: make install [WEBOTS_VERSION=R2025a] [INSTALL_DIR=/usr/local/webots]"
	@echo "Instala o Webots, dependências e configura o PATH."
	@echo "Após a instalação, feche e reabra o terminal."

install: dependencies download extract configure
	@echo "--- Instalação do Webots $(WEBOTS_VERSION) concluída com sucesso! ---"
	@echo "FECHE E REABRA SEU TERMINAL para que o Webots esteja disponível no PATH."
	@echo "Lembre-se de verificar seus drivers gráficos (GPU)."

dependencies:
	@echo "--- Instalando dependências do sistema (via apt)... ---"
	@sudo apt update > /dev/null || { echo "ERRO: Falha ao atualizar repositórios apt."; exit 1; }
	@sudo apt install -y wget tar libgl1-mesa-glx libglu1-mesa-dev libfontconfig1 libxkbcommon-x11-0 > /dev/null || { echo "ERRO: Falha ao instalar dependências."; exit 1; }

download:
	@echo "--- Baixando Webots $(WEBOTS_VERSION)... ---"
	@wget -O $(DOWNLOAD_FILE) $(DOWNLOAD_URL) > /dev/null || { echo "ERRO: Falha ao baixar Webots."; exit 1; }
ifdef DOWNLOAD_CHECKSUM
	@echo "--- Verificando checksum... ---"
	@echo "$(DOWNLOAD_CHECKSUM)  $(DOWNLOAD_FILE)" > $(CHECKSUM_FILE)
	@sha256sum --check $(CHECKSUM_FILE) > /dev/null || { \
		echo "ERRO: Checksum do arquivo não confere!"; \
		rm -f $(CHECKSUM_FILE); \
		exit 1; \
	}
	@rm -f $(CHECKSUM_FILE)
else
	@echo "--- Pulando verificação de checksum. ---"
endif

extract:
	@echo "--- Extraindo Webots para $(INSTALL_DIR)... ---"
	@sudo mkdir -p $(INSTALL_DIR) || { echo "ERRO: Falha ao criar diretório de instalação."; exit 1; }
	@sudo tar -xjf $(DOWNLOAD_FILE) -C $(INSTALL_DIR) --strip-components=1 > /dev/null || { echo "ERRO: Falha ao extrair o arquivo."; exit 1; }

configure:
	@echo "--- Configurando PATH do Webots... ---"
	# Configura o script em /etc/profile.d/ (para sessões de login e terminais que o carregam)
	@echo "export WEBOTS_HOME=\"$(INSTALL_DIR)\"" | sudo tee $(PROFILE_SCRIPT) > /dev/null
	@echo "export PATH=\"$$PATH:$$WEBOTS_HOME\"" | sudo tee -a $(PROFILE_SCRIPT) > /dev/null

	# Adiciona 'source' ao .bashrc ou .zshrc (para sessões não-login, mais comuns)
	@if [ -f ~/.bashrc ]; then \
		if ! grep -q "^source $(PROFILE_SCRIPT)" ~/.bashrc; then \
			echo -e "\n# Webots PATH (adicionado por Makefile)\nsource $(PROFILE_SCRIPT)" >> ~/.bashrc; \
			echo "Linha 'source $(PROFILE_SCRIPT)' adicionada ao ~/.bashrc."; \
		fi; \
	elif [ -f ~/.zshrc ]; then \
		if ! grep -q "^source $(PROFILE_SCRIPT)" ~/.zshrc; then \
			echo -e "\n# Webots PATH (adicionado por Makefile)\nsource $(PROFILE_SCRIPT)" >> ~/.zshrc; \
			echo "Linha 'source $(PROFILE_SCRIPT)' adicionada ao ~/.zshrc."; \
		fi; \
	else \
		echo "AVISO: Não foi possível configurar automaticamente o PATH em ~/.bashrc ou ~/.zshrc."; \
		echo "Por favor, adicione 'source $(PROFILE_SCRIPT)' manualmente ao seu arquivo de configuração de shell."; \
	fi

clean:
	@echo "--- Removendo arquivo de download temporário... ---"
	@rm -f $(DOWNLOAD_FILE)
