# Makefile para Instalação do Webots [EDROM-2025]
# Focado em download, extração e configuração automática do PATH.

# --- Variáveis de Configuração ---
# Você pode sobrescrever estas variáveis ao chamar o make.
# Ex: make install WEBOTS_VERSION=R2023b
WEBOTS_VERSION ?= R2025a
INSTALL_DIR ?= /usr/local/webots
DOWNLOAD_URL = https://github.com/cyberbotics/webots/releases/download/$(WEBOTS_VERSION)/webots-$(WEBOTS_VERSION)-x86-64.tar.bz2

# Checksum SHA256 do arquivo TAR.BZ2 (HEXADECIMAL). ALTAMENTE RECOMENDADO!
DOWNLOAD_CHECKSUM ?= c5127fb4206c57a5ae5523f1b7f3da8b670bc8926d9ae08595e139f226f38c38

# --- Variáveis Internas (Não modifique) ---
DOWNLOAD_FILE = /tmp/webots-$(WEBOTS_VERSION).tar.bz2
SUDO = $(shell which sudo)
BASH_CONFIG = $(HOME)/.bashrc
ZSH_CONFIG = $(HOME)/.zshrc

# --- Define a lógica de verificação de checksum condicionalmente ---
# Isso é processado pelo 'make' antes de executar qualquer receita.
ifdef DOWNLOAD_CHECKSUM
  # Comandos a serem executados se DOWNLOAD_CHECKSUM estiver definido.
  CHECKSUM_LOGIC = \
	echo "--- Verificando integridade do arquivo (checksum)..."; \
	echo "$(DOWNLOAD_CHECKSUM)  $(DOWNLOAD_FILE)" | sha256sum --check --status || { \
		echo "ERRO: Checksum do arquivo não confere! O download pode estar corrompido."; \
		exit 1; \
	}; \
	echo "INFO: Checksum verificado com sucesso."
else
  # Comando a ser executado se DOWNLOAD_CHECKSUM NÃO estiver definido.
  CHECKSUM_LOGIC = echo "AVISO: Pulando verificação de checksum."
endif


# --- Targets ---

.PHONY: all install dependencies download extract configure clean help

# Target padrão: exibe a ajuda.
all: help

help:
	@echo ""
	@echo "Uso: make <target>"
	@echo "--------------------------------------------------------------------------"
	@echo "  Targets disponíveis:"
	@echo "    install      - Executa todo o processo: dependências, download, extração e configuração."
	@echo "    dependencies - Instala apenas as dependências do sistema via APT."
	@echo "    download     - Baixa e verifica o arquivo de instalação do Webots."
	@echo "    extract      - Extrai o Webots para o diretório de instalação."
	@echo "    configure    - Adiciona a configuração do Webots ao seu PATH (em .bashrc/.zshrc)."
	@echo "    clean        - Remove os arquivos temporários de download."
	@echo "    help         - Exibe esta mensagem de ajuda."
	@echo ""
	@echo "  Este Makefile configura o PATH automaticamente ao executar 'make install'."
	@echo ""

install: dependencies download extract configure
	@echo ""
	@echo "**************************************************************************"
	@echo "*** ***"
	@echo "*** Instalação do Webots $(WEBOTS_VERSION) concluída com sucesso!     ***"
	@echo "*** ***"
	@echo "**************************************************************************"
	@echo ""
	@echo ">>> AÇÃO NECESSÁRIA: Recarregue seu terminal <<<"
	@echo ""
	@echo "O PATH foi configurado no seu arquivo .bashrc ou .zshrc."
	@echo "Para que o comando 'webots' funcione, você precisa ABRIR UM NOVO TERMINAL."
	@echo ""
	@echo "Ou, execute UMA VEZ no seu terminal atual:"
	@echo "   source \$$HOME/.bashrc   (ou source \$$HOME/.zshrc se você usa Zsh)"
	@echo ""

dependencies:
	@echo "--- [1/4] Instalando dependências do sistema (requer sudo)... ---"
	@$(SUDO) apt-get update -qq || { echo "ERRO: Falha ao atualizar repositórios apt."; exit 1; }
	@$(SUDO) apt-get install -y -qq wget tar libgl1-mesa-glx libglu1-mesa-dev libfontconfig1 libxkbcommon-x11-0 || { echo "ERRO: Falha ao instalar dependências."; exit 1; }
	@echo "INFO: Dependências instaladas."

download:
	@echo "--- [2/4] Baixando Webots $(WEBOTS_VERSION)... ---"
	@wget --progress=bar:force -O $(DOWNLOAD_FILE) $(DOWNLOAD_URL) 2>&1 | grep --line-buffered "%" || { echo "ERRO: Falha ao baixar Webots."; exit 1; }
	@$(CHECKSUM_LOGIC)

extract:
	@echo "--- [3/4] Extraindo Webots para $(INSTALL_DIR) (requer sudo)... ---"
	@$(SUDO) mkdir -p $(INSTALL_DIR) || { echo "ERRO: Falha ao criar diretório de instalação."; exit 1; }
	@echo "INFO: Extraindo, isso pode levar um momento..."
	@$(SUDO) tar -xjf $(DOWNLOAD_FILE) -C $(INSTALL_DIR) --strip-components=1 || { echo "ERRO: Falha ao extrair o arquivo."; exit 1; }
	@echo "INFO: Arquivos extraídos."

configure:
	@echo "--- [4/4] Configurando o ambiente para o Webots... ---"
	@if [ -f "$(BASH_CONFIG)" ]; then \
		if ! grep -q 'export PATH="$(INSTALL_DIR)' $(BASH_CONFIG); then \
			echo "" >> $(BASH_CONFIG); \
			echo '# Configuração do Webots adicionada pelo Makefile' >> $(BASH_CONFIG); \
			echo 'export PATH="$(INSTALL_DIR):$$PATH"' >> $(BASH_CONFIG); \
			echo "INFO: Configuração do Webots adicionada ao $(BASH_CONFIG)"; \
		else \
			echo "INFO: Configuração do Webots já existe em $(BASH_CONFIG). Nenhuma alteração feita."; \
		fi \
	elif [ -f "$(ZSH_CONFIG)" ]; then \
		if ! grep -q 'export PATH="$(INSTALL_DIR)' $(ZSH_CONFIG); then \
			echo "" >> $(ZSH_CONFIG); \
			echo '# Configuração do Webots adicionada pelo Makefile' >> $(ZSH_CONFIG); \
			echo 'export PATH="$(INSTALL_DIR):$$PATH"' >> $(ZSH_CONFIG); \
			echo "INFO: Configuração do Webots adicionada ao $(ZSH_CONFIG)"; \
		else \
			echo "INFO: Configuração do Webots já existe em $(ZSH_CONFIG). Nenhuma alteração feita."; \
		fi \
	else \
		echo "AVISO: Não foi possível encontrar ~/.bashrc ou ~/.zshrc para configuração automática."; \
	fi

clean:
	@echo "--- Removendo arquivo de download temporário... ---"
	@rm -f $(DOWNLOAD_FILE)
	@echo "INFO: Limpeza concluída."

