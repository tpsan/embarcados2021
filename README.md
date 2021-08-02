# Projeto de acionamento de motor e comunicação com encoder para a movimentação ponto a ponto de um eixo de robo de reabilitação de um grau de liberdade

Para encontrar o projeto, basta acessar a pasta /Projeto-braco e para o devido funcionamento dos códigos contidos nesta pasta, é necessária a instalação do CANopen.

<hr>

Por meio do Script config.sh teremos o CAN Bus virtual e poderemos inicializar os 3 nós para o uso do comando cocomm.

As threads foram utilizadas para permitir a comunicação e é possível integrar os scripts que usam essas threads aos códigos em C.


# Configuração do linux da Toradex Colibri VFxx para a habilitação das conexões CANtx

Faça o download da toolchain que para permitir a compilação no formato da Colibri.

```sh
$ cd <pasta que você desejar deixar salvo>
$ wget -O gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz "https://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/arm-linux-gnueabihf/gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz"
$ tar xvf gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf.tar.xz
$ ln -s gcc-linaro-7.3.1-2018.05-x86_64_arm-linux-gnueabihf gcc-linaro
$ nano export_compiler
# digita
	export ARCH=arm
	export PATH=~/gcc-linaro/bin/:$PATH
	export CROSS_COMPILE=arm-linux-gnueabihf-
#ctrl+x s enter, para salvar
$ source export_compiler
```

Agora basta baixar e editar os arquivos que serão ser compilados e definir o Kernel.

```sh
$ cd <pasta onde você desejar>
#No nosso caso <branch> foi tordex_vf_4.4, mas isso depende da sua versão, isso é explicado mais a fundo em developer.toradex.com
$ git clone -b <branch> git://git.toradex.com/linux-toradex.git
```
Abra o seguinte endereço: ~/linux-toradex/arch/arm/boot/dts/vf-colibri-eval-v3.dtsi 

<ol>
<li>Altere o status do i2c0 de "okay" para "disabled" (linha 136)</li>
<li>Adicione as linhas a seguir acima de "&i2c0 {" (linha 135)
	<p> &can0 {<br>
       	status = "okay";<br>
				};</p></li>
<li>Agora digite no terminal:
<p>~$ cd linux-toradex <br>
~/linux-toradex/ make colibri_vf_defconfig
~/linux-toradex/ make
</p>
</li>
</ol>

<hr>

# Configuração de arquivos do CANopenNode

Para fazer o CANopenNode funcionar vamos usar os comandos a seguir:
```sh
~$ cd embarcados2021
~/embarcados2021/ git init
~/embarcados2021/ git pull
~/embarcados2021/ git submodule update --init --recursive
~/embarcados2021/ cd CANopenDemo/CANopenLinux/cocomm
~/embarcados2021/CANopenDemo/CANopenLinux/cocomm/ make
~/embarcados2021/CANopenDemo/CANopenLinux/cocomm/ sudo make install //Adicionando a função cocomm no Terminal
~/embarcados2021/CANopenDemo/CANopenLinux/ make
~/embarcados2021/CANopenDemo/CANopenLinux/ sudo make install
Função canopend: 
~/embarcados2021/CANopenDemo/CANopenLinux/cocomm/ cd ../demo
~/embarcados2021/CANopenDemo/demo/ nano Makefile
# Serão adicionadas as seguintes linhas no final:
'install:
	cp $(LINK_TARGET) /usr/bin/$(LINK_TARGET)'
~/embarcados2021/CANopenDemo/demo/ make
~/embarcados2021/CANopenDemo/demo/ sudo make install
Função demoLinuxDevice.
```


<i>"[...] By default device uses Object Dictionary from `CANopenNode/example`, which contains only communication parameters." ~ embarcados2021/CANopenDemo/CANopenLinux/README.md</i>

Para criar um nó para a rede CAN é necessário utilizar o OD, não sendo necessário, portanto alterar os aquivos de cima. A modificação no OD garante maior liberdade para as capacidades dos nós.

<i>/CANopenDemo/demo/OD.c</i>

Por fim, basta alterar os valores dos parâmetros de comunicação PDO e os de mapeamento PDO.  valores 1400(4) e 1800(4) seguido de 1600(4) e 1A00(4) respectivamente. 



<hr>

# Testando o CAN

Para a comunicação CAN é necessária a instalação das seguintes ferramentas:

CANopen:
https://opensource.lely.com/canopen/docs/installation/

```sh
~$ sudo apt-get update
~$ git clone https://github.com/CANopenNode/CANopenDemo.git
~$ cd CANopenDemo
~/CANopenDemo/$ git submodule update --init --recursive
```
É necessário rodar em pelo menos 4 terminais diferentes: 

> Terminal 1
```sh
# Função de comunicação
~$ cd CANopenDemo/CANopenLinux/cocomm/
~CANopenDemo/CANopenLinux/cocomm/$ make
~CANopenDemo/CANopenLinux/cocomm/$ sudo make install
~CANopenDemo/CANopenLinux/cocomm/$ cd
# Início do barramento virtual can0 
~$ sudo modprobe vcan
~$ sudo ip link add dev can0 type vcan
~$ sudo ip link set up can0
#Recepção dos envios do barramento 
~$ candump can0
```

> Terminal 2
```sh
# Criação de nó "master"
~$ cd CANopenDemo/CANopenLinux/
~/CANopenDemo/CANopenLinux/$ make
~/CANopenDemo/CANopenLinux/$ rm *.persist
~/CANopenDemo/CANopenLinux/$ ./canopend can0 -i 1 -c "local-/tmp/CO_command_socket"
# É possível a criação de vários nós aqui  "./canopend (Barramento -i id-Nó)"
```

> Terminal 3
```sh
# Nó "servo"
~$ cd CANopenDemo/demo/
~/CANopenDemo/demo/$ make
~/CANopenDemo/demo/$ rm *.persist
~/CANopenDemo/demo/$ ./demoLinuxDevice can0
```

> Terminal 4
```sh
#Transferência de commandos via cocomm
~$ cocomm "help"
```

<hr>

# Links Possivelmente úteis

> Git com os arquivos do CANopenDemo, de onde fizemos o git clone e de onde baseamos este projeto:
https://github.com/CANopenNode/CANopenDemo

>Onde vimos como usar o CAN no Colibri:
https://developer.toradex.com/knowledge-base/can-controller-area-network-on-colibri-module#CAN_on_Colibri_VFxx

> Habilitação do CAN na toradex Colibri VFxx <br>
https://developer.toradex.com/knowledge-base/can-linux#Kernel_Support_Colibri_VFxx

> Criação da Build do kernel e onde conseguimos os arquivos do git toradex <br>
https://developer.toradex.com/knowledge-base/build-u-boot-and-linux-kernel-from-source-code
