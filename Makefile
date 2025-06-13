# Makefile para Instalação do Webots [EDROM-2025]
# Focado em download e extração, com instruções para configuração manual do PATH.

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

.PHONY: all install dependencies download extract clean help

# Target padrão: exibe a ajuda.
all: help

help:
	@echo ""
	@echo "Uso: make <target>"
	@echo "--------------------------------------------------------------------------"
	@echo "  Targets disponíveis:"
	@echo "    install      - Executa o processo de download e extração."
	@echo "    dependencies - Instala apenas as dependências do sistema via APT."
	@echo "    download     - Baixa e verifica o arquivo de instalação do Webots."
	@echo "    extract      - Extrai o Webots para o diretório de instalação."
	@echo "    clean        - Remove os arquivos temporários de download."
	@echo "    help         - Exibe esta mensagem de ajuda."
	@echo ""
	@echo "  Este Makefile NÃO configura o PATH automaticamente."
	@echo "  Ele irá exibir as instruções necessárias ao final da instalação."
	@echo ""

install: dependencies download extract
	@echo ""
	@echo "**************************************************************************"
	@echo "*** ***"
	@echo "*** Instalação do Webots $(WEBOTS_VERSION) concluída com sucesso!     ***"
	@echo "*** ***"
	@echo "**************************************************************************"
	@echo ""
	@echo ">>> AÇÃO NECESSÁRIA: Configure seu ambiente manualmente <<<"
	@echo ""
	@echo "O Webots foi instalado em: $(INSTALL_DIR)"
	@echo "Para que o comando 'webots' funcione, adicione o diretório ao seu PATH."
	@echo ""
	@echo "1. Para usar o Webots APENAS na sessão atual do terminal, execute:"
	@echo "   export PATH=\"$(INSTALL_DIR):\$$PATH\""
	@echo ""
	@echo "2. Para tornar a mudança PERMANENTE (recomendado), adicione a linha"
	@echo "   abaixo ao final do seu arquivo de configuração de shell:"
	@echo "   (Normalmente ~/.bashrc ou ~/.zshrc)"
	@echo ""
	@echo "   echo 'export PATH=\"$(INSTALL_DIR):\$$PATH\"' >> \$$HOME/.bashrc"
	@echo ""
	@echo "   Depois, recarregue a configuração com 'source \$$HOME/.bashrc' ou abra um novo terminal."
	@echo ""

dependencies:
	@echo "--- [1/3] Instalando dependências do sistema (requer sudo)... ---"
	@$(SUDO) apt-get update -qq || { echo "ERRO: Falha ao atualizar repositórios apt."; exit 1; }
	@$(SUDO) apt-get install -y -qq wget tar libgl1-mesa-glx libglu1-mesa-dev libfontconfig1 libxkbcommon-x11-0 || { echo "ERRO: Falha ao instalar dependências."; exit 1; }
	@echo "INFO: Dependências instaladas."

download:
	@echo "--- [2/3] Baixando Webots $(WEBOTS_VERSION)... ---"
	@wget --progress=bar:force -O $(DOWNLOAD_FILE) $(DOWNLOAD_URL) 2>&1 | grep --line-buffered "%" || { echo "ERRO: Falha ao baixar Webots."; exit 1; }
	@$(CHECKSUM_LOGIC)

extract:
	@echo "--- [3/3] Extraindo Webots para $(INSTALL_DIR) (requer sudo)... ---"
	@$(SUDO) mkdir -p $(INSTALL_DIR) || { echo "ERRO: Falha ao criar diretório de instalação."; exit 1; }
	@echo "INFO: Extraindo, isso pode levar um momento..."
	@$(SUDO) tar -xjf $(DOWNLOAD_FILE) -C $(INSTALL_DIR) --strip-components=1 || { echo "ERRO: Falha ao extrair o arquivo."; exit 1; }
	@echo "INFO: Arquivos extraídos."

clean:
	@echo "--- Removendo arquivo de download temporário... ---"
	@rm -f $(DOWNLOAD_FILE)
	@echo "INFO: Limpeza concluída."

