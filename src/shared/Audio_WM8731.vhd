--
-- Multicore 2 / Multicore 2+
--
-- Copyright (c) 2017-2020 - Victor Trucco
--
-- All rights reserved
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
		
-- Abstracao do audio para chip WM8731

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity Audio_WM8731 is
	port (
		reset_i			: in    std_logic;							-- Reset geral
		clock_i			: in    std_logic;							-- Clock 24 MHz
		mic_i				: in    std_logic;							-- Entrada 1 bit MIC
		spk_i				: in    std_logic;							-- Entrada 1 bit Speaker
		ear_o				: out   std_logic;							-- Saida 1 bit EAR

		i2s_xck_o		: out   std_logic;							-- Ligar nos pinos do TOP
		i2s_bclk_o		: out   std_logic;
		i2s_adclrck_o	: out   std_logic;
		i2s_adcdat_i	: in    std_logic;
		i2s_daclrck_o	: out   std_logic;
		i2s_dacdat_o	: out   std_logic;
		
		i2c_sda_io		: inout std_logic;							-- Ligar no pino I2C SDA
		i2c_scl_io		: inout std_logic								-- Ligar no pino I2C SCL
	);
end entity;

architecture rtl of Audio_WM8731 is

	signal pcm_lrclk			: std_logic;
	signal pcm_outl			: std_logic_vector(15 downto 0);
	signal pcm_outr			: std_logic_vector(15 downto 0);
	signal pcm_inl				: std_logic_vector(15 downto 0);
	signal pcm_inr				: std_logic_vector(15 downto 0);

	signal ear_w				: std_logic;
	signal spk_s				: std_logic_vector(15 downto 0);
	signal mic_s				: std_logic_vector(15 downto 0);
	signal ear_s				: std_logic_vector(15 downto 0);

	constant spk_volume		: std_logic_vector(15 downto 0) := "0011000000000000";
	constant mic_volume		: std_logic_vector(15 downto 0) := "0000100000000000";
	constant ear_volume		: std_logic_vector(15 downto 0) := "0000001000000000";

begin

	i2s: entity work.i2s_intf
	generic map (
		mclk_rate	=> 12000000,
		sample_rate	=> 96000,
		preamble		=>  1, -- I2S
		word_length	=> 16
	)
	port map (
		-- 2x MCLK in (e.g. 24 MHz for WM8731 USB mode)
		clock_i				=> clock_i,
		reset_i				=> reset_i,
		-- Parallel IO
		pcm_inl_o			=> pcm_inl,
		pcm_inr_o			=> open, --pcm_inr,
		pcm_outl_i			=> pcm_outl,
		pcm_outr_i			=> pcm_outr,
		-- Codec interface (right justified mode)
		-- MCLK is generated at half of the CLK input
		i2s_mclk_o			=> i2s_xck_o,
		-- LRCLK is equal to the sample rate and is synchronous to
		-- MCLK.  It must be related to MCLK by the oversampling ratio
		-- given in the codec datasheet.
		i2s_lrclk_o			=> pcm_lrclk,
		-- Data is shifted out on the falling edge of BCLK, sampled
		-- on the rising edge.  The bit rate is determined such that
		-- it is fast enough to fit preamble + word_length bits into
		-- each LRCLK half cycle.  The last cycle of each word may be 
		-- stretched to fit to LRCLK.  This is OK at least for the 
		-- WM8731 codec.
		-- The first falling edge of each timeslot is always synchronised
		-- with the LRCLK edge.
		i2s_bclk_o			=> i2s_bclk_o,
		-- Output bitstream
		i2s_d_o				=> i2s_dacdat_o,
		-- Input bitstream
		i2s_d_i				=> i2s_adcdat_i
	);

	i2s_adclrck_o <= pcm_lrclk;
	i2s_daclrck_o <= pcm_lrclk;
	 

	i2c: entity work.i2c_loader 
	generic map (
		device_address	=> 16#1a#,		-- Address of slave to be loaded
		num_retries		=> 0,				-- Number of retries to allow before stopping
		-- Length of clock divider in bits.  Resulting bus frequency is
		-- CLK/2^(log2_divider + 2)
		log2_divider	=> 7
	)
	port map (
		clock_i			=> clock_i,
		reset_i			=> reset_i,
		i2c_scl_io		=> i2c_scl_io,
		i2c_sda_io		=> i2c_sda_io,
		is_done_o		=> open,
		is_error_o		=> open
	);

	ear_o	<= ear_w;

	spk_s <= spk_volume when spk_i = '1'							else (others => '0');
	mic_s <= mic_volume when mic_i = '1'							else (others => '0');
	ear_s <= ear_volume when ear_w = '1' 						 	else (others => '0');


	pcm_outl <= std_logic_vector(unsigned(spk_s) + unsigned(mic_s) + unsigned(ear_s));
	pcm_outr <= std_logic_vector(unsigned(spk_s) + unsigned(mic_s) + unsigned(ear_s));

	-- Hysteresis for EAR input (should help reliability)
	process (clock_i)
		variable in_val : integer;
	begin		
		if rising_edge(clock_i) then
			in_val := to_integer(signed(pcm_inl));
			if in_val < -15 then
				ear_w <= '0';
			elsif in_val > 15 then
				ear_w <= '1';
			end if;
		end if;
	end process;

end architecture;